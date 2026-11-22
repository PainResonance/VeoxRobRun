#include "pch.h"
#include "Memory.h"
#include "CheatFunction.h"
#include "CameraFinder.h"
#include "RobRunReader.h"
#include "ConfigFile.h"
#include "ConfigInstance.h"
#include "PlayerEsp.h"
#include "WorldEsp.h"
#include "Esp/Radar.h"
#include "Esp/WebRadar.h"
#include "DebugLog.h"
#include "Esp/WeaponSprites.h"
#include "Esp/IconSprites.h"
#include "Aim/AimController.h"
#include "SDK/SkinChanger.h"
#include "SDK/FovChanger.h"
#include "Input/InputManager.h"
#include "Input/DeviceSession.h"
#include "UI/VeoxMenu.h"
#include "ImGUI/imgui.h"
#include "Overlay/Overlay.h"
#include "Overlay/Backdrop.h"
#include <atomic>
#include <mutex>
#include <cstdio>
#include <cstdarg>
#include <stdexcept>
#include <eh.h>
#include <shellapi.h>
#include <cmath>
#pragma comment(lib, "shell32.lib")

static std::atomic<bool> g_attached{ false };
static std::atomic<bool> g_stopAttach{ false };
static std::atomic<bool> g_pendingAttachToast{ false };
static std::atomic<bool> g_pendingReattach{ false };
static std::thread g_attachThread;
static HINSTANCE g_hInstance = nullptr;

// ---------------------------------------------------------------------------
// Crash trail. A release build has no symbols to hand back from the second PC,
// so every stage is written to veox-diag.log beside the exe and an access
// violation is translated into a C++ exception the existing handlers print.
// ---------------------------------------------------------------------------
static std::mutex g_diagMutex;
static std::atomic<const char*> g_stage{ "boot" };

static void SetStage(const char* stage)
{
	g_stage.store(stage, std::memory_order_release);
}

void VeoxSetStage(const char* stage)
{
	SetStage(stage);
}

static void Diag(const char* fmt, ...);

// Public form for other translation units (the ESP logs a snapshot probe).
void VeoxDiag(const char* fmt, ...)
{
	char line[1024];
	va_list args;
	va_start(args, fmt);
	vsnprintf(line, sizeof(line), fmt, args);
	va_end(args);

	wchar_t exePath[MAX_PATH]{};
	GetModuleFileNameW(nullptr, exePath, MAX_PATH);
	std::wstring path(exePath);
	const auto pos = path.find_last_of(L"\\/");
	if (pos != std::wstring::npos) path.resize(pos + 1);
	path += L"veox-diag.log";

	std::lock_guard<std::mutex> lock(g_diagMutex);
	if (FILE* f = _wfopen(path.c_str(), L"a")) {
		SYSTEMTIME st{};
		GetLocalTime(&st);
		fprintf(f, "[%02d:%02d:%02d] %s\n", st.wHour, st.wMinute, st.wSecond, line);
		fclose(f);
	}
	printf("%s\n", line);
}

static void Diag(const char* fmt, ...)
{
	char line[1024];
	va_list args;
	va_start(args, fmt);
	vsnprintf(line, sizeof(line), fmt, args);
	va_end(args);
	VeoxDiag("%s", line);
}

static void SeTranslator(unsigned int code, EXCEPTION_POINTERS* info)
{
	char text[512];
	snprintf(text, sizeof(text), "FAULT code=0x%08X addr=%p stage=%s",
		code,
		(info && info->ExceptionRecord) ? info->ExceptionRecord->ExceptionAddress : nullptr,
		g_stage.load());
	Diag("%s", text);
	throw std::runtime_error(text);
}

static std::mutex g_statusMutex;
static std::string g_attachStatus = "Waiting for DMA card...";

// Serialises reader lifetime: Init() (attach thread) and Shutdown() (render
// thread on reattach/exit) reset the same Il2Cpp/RobRunCamera/CameraFinder
// statics, and the render thread must never be reading them while that runs.
static std::mutex g_lifecycleMutex;

static void SetAttachStatus(const std::string& text)
{
	std::lock_guard<std::mutex> lock(g_statusMutex);
	g_attachStatus = text;
}

static std::string GetAttachStatus()
{
	std::lock_guard<std::mutex> lock(g_statusMutex);
	return g_attachStatus;
}

static bool AttachGame()
{
	SetStage("attach:process");
	if (!TargetProcess.Init("ROB RUN.exe"))
	{
		SetAttachStatus("Waiting for ROB RUN...");
		return false;
	}

	SetStage("attach:modules");
	size_t gaBase = TargetProcess.GetBaseAddress("GameAssembly.dll");
	size_t unityBase = TargetProcess.GetBaseAddress("UnityPlayer.dll");
	if (!gaBase || !unityBase)
	{
		SetAttachStatus("Failed to find game modules.");
		return false;
	}

	SetStage("attach:reader");
	{
		std::lock_guard<std::mutex> lifecycle(g_lifecycleMutex);
		CameraFinder::Init(unityBase);
		RobRunReader::Init(gaBase, unityBase);
		SkinChanger::Start();
	}
	printf("GameAssembly @ 0x%llX | UnityPlayer @ 0x%llX\n", gaBase, unityBase);
	Diag("attached: GameAssembly=0x%llX UnityPlayer=0x%llX", (unsigned long long)gaBase, (unsigned long long)unityBase);
	// Resolve the game PC's gafAsyncKeyState once the DMA process context is
	// live. This can take a few seconds (kernel module scans), so it belongs on
	// the attach thread. Without it every aim key read returns false.
	SetStage("attach:keyboard");
	const bool keyboardReady = Keyboard::InitKeyboard();
	printf("[Input] game PC keyboard %s\n",
		keyboardReady ? "ready" : "unavailable (host/device input only)");
	SetStage("attach:done");
	Diag("keyboard %s", keyboardReady ? "ready" : "unavailable");
	SetAttachStatus("Attached.");
	g_attached.store(true, std::memory_order_release);
	g_pendingAttachToast.store(true);
	Aim::StartAimWorker();
	return true;
}

static void StartAttachLoop()
{
	g_stopAttach.store(false);
	g_attachThread = std::thread([] {
		// The SEH translator is per-thread; this one is not the WinMain thread.
		_set_se_translator(SeTranslator);
		while (!g_stopAttach.load()) {
			try {
				if (AttachGame()) return;
			} catch (const std::exception& e) {
				Diag("attach thread exception: %s", e.what());
			} catch (...) {
				Diag("attach thread unknown exception (stage=%s)", g_stage.load());
			}
			for (int i = 0; i < 30 && !g_stopAttach.load(); ++i)
				Sleep(100);
		}
	});
}

static void StopAttachLoop()
{
	g_stopAttach.store(true);
	if (g_attachThread.joinable()) g_attachThread.join();
}

void RequestReattach()
{
	g_pendingReattach.store(true);
}

static void TickOverlay()
{
	if (g_pendingReattach.exchange(false))
	{
		Overlay::Notify("Re-attaching...", 0xFFFFCC44u, 2.f);
		// Drop the render thread's reader access before tearing anything down:
		// RefreshCamera() and the draw callbacks are gated on this flag.
		g_attached.store(false, std::memory_order_release);
		Aim::StopAimWorker();
		// Join the attach thread before taking the lifecycle lock. It may be
		// inside AttachGame holding that lock, so joining while holding it
		// would deadlock.
		StopAttachLoop();
		// An attach that completed during that join may have flipped the flag
		// back on and restarted the aim worker; undo both before shutdown.
		g_attached.store(false, std::memory_order_release);
		Aim::StopAimWorker();
		{
			std::lock_guard<std::mutex> lifecycle(g_lifecycleMutex);
			SkinChanger::Stop();
			FovChanger::Reset();
			RobRunReader::Shutdown();
			TargetProcess.ResetProcess();
		}
		SetAttachStatus("Re-attaching...");
		StartAttachLoop();
	}

	const ImVec2 display = ImGui::GetIO().DisplaySize;
	Aim::SetDisplaySize(display.x, display.y);
	Radar::ProcessPendingMap();
	// Camera polling runs on RobRunReader's dedicated high-priority thread so
	// the render thread never stalls waiting on DMA transactions.

	if (g_pendingAttachToast.exchange(false))
		Overlay::Notify("Attached to ROB RUN", 0xFF8CDC64u, 3.f);

	// Battle mode suppresses world ESP/radar/money HUD so only player ESP is
	// shown. Toggle on the rising edge of the selected key ("Always" is not a
	// toggle key). Ignore while the menu is open to avoid accidental toggles
	// when changing the binding.
	static bool battleKeyWasHeld = false;
	if (!Overlay::MenuOpen()) {
		const int key = ConfigInstance.Overlay.BattleModeKey;
		const bool held = key >= 0 && key < Aim::KeyCount() && !Aim::IsAlwaysKey(key) && Aim::IsKeyHeld(key);
		if (held && !battleKeyWasHeld) {
			ConfigInstance.Overlay.BattleMode = !ConfigInstance.Overlay.BattleMode;
			Overlay::Notify(ConfigInstance.Overlay.BattleMode
				? "Battle Mode: ON (player ESP only)"
				: "Battle Mode: OFF", 0xFF8CDC64u, 2.f);
		}
		battleKeyWasHeld = held;
	} else {
		battleKeyWasHeld = false;
	}

	// Persist settings whenever the menu closes, so a user does not have to
	// remember a separate save step. Shutdown saves again as a backstop.
	static bool wasMenuOpen = false;
	const bool menuOpen = Overlay::MenuOpen();
	if (wasMenuOpen && !menuOpen)
		ConfigFile::Save();
	wasMenuOpen = menuOpen;
}

static std::atomic<bool> g_shutdownComplete{ false };

static BOOL WINAPI ConsoleCtrlHandler(DWORD ctrlType)
{
	switch (ctrlType) {
	case CTRL_C_EVENT:
	case CTRL_BREAK_EVENT:
	case CTRL_CLOSE_EVENT:
	case CTRL_SHUTDOWN_EVENT:
		// Signal the overlay loop to exit cleanly so worker threads stop and
		// VMMDLL_Close() runs instead of abandoning an in-flight FPGA read.
		Overlay::RequestExit();
		for (int i = 0; i < 40 && !g_shutdownComplete.load(std::memory_order_acquire); ++i)
			Sleep(50);
		return TRUE;
	default:
		return FALSE;
	}
}

static RobRunCamera::CameraState DemoCamera()
{
	RobRunCamera::CameraState camera{};
	camera.valid = true;
	camera.position = {0.f, 1.6f, 0.f};
	camera.forward = {0.f, 0.f, 1.f};
	camera.right = {1.f, 0.f, 0.f};
	camera.up = {0.f, 1.f, 0.f};
	return camera;
}

static RobRunReader::SnapshotPtr MakeDemoSnapshot(float seconds)
{
	auto snapshot = std::make_shared<RobRunReader::Snapshot>();
	snapshot->camera = DemoCamera();
	snapshot->localEntityId = 0;
	snapshot->localPosition = {0.f, 1.6f, 0.f};
	const char* names[] = {"Demo Alex", "Demo Sam", "Demo Riley"};
	const char* roles[] = {"Criminal", "Police Officer", "Civilian"};
	for (int i = 0; i < 3; ++i) {
		RobRunReader::EntityData player{};
		player.entityId = i + 1;
		const float phase = seconds * 1.4f + i;
		player.position = {-3.f + 3.f * i + std::sin(phase) * 0.5f, 0.f, 10.f + i * 1.5f};
		player.nicknameUtf8 = names[i];
		player.roleName = roles[i];
		player.health = 100.f - 25.f * i;
		player.maxHealth = 100.f;
		player.healthKnown = true;
		player.visible = i != 2;
		player.hasArmor = i == 1;
		player.armor = player.hasArmor ? 50.f : 0.f;
		player.stamina = 80.f - 20.f * i;
		player.lookDirection = {0.f, 0.f, -1.f};
		auto joint = [&](float x, float y, float z = 0.f) {
			return player.position + Vector3{x, y, z};
		};
		auto& pose = player.bonePoints;
		const float stride = std::sin(phase * 3.f) * 0.14f;
		pose.head = joint(0.f, 1.72f);
		pose.neck = joint(0.f, 1.51f);
		pose.chest = joint(0.f, 1.32f);
		pose.pelvis = joint(0.f, 0.92f);
		pose.leftShoulder = joint(-0.22f, 1.41f);
		pose.rightShoulder = joint(0.22f, 1.41f);
		pose.leftElbow = joint(-0.35f, 1.15f, stride);
		pose.rightElbow = joint(0.35f, 1.15f, -stride);
		pose.leftHand = joint(-0.37f, 0.94f, stride);
		pose.rightHand = joint(0.37f, 0.94f, -stride);
		pose.leftHip = joint(-0.13f, 0.9f);
		pose.rightHip = joint(0.13f, 0.9f);
		pose.leftKnee = joint(-0.15f, 0.46f, -stride);
		pose.rightKnee = joint(0.15f, 0.46f, stride);
		pose.leftFoot = joint(-0.15f, 0.03f, -stride);
		pose.rightFoot = joint(0.15f, 0.03f, stride);
		pose.hasHead = pose.hasNeck = pose.hasChest = pose.hasPelvis = pose.valid = true;
		player.bones = {
			{pose.head, pose.neck}, {pose.neck, pose.chest}, {pose.chest, pose.pelvis},
			{pose.chest, pose.leftShoulder}, {pose.leftShoulder, pose.leftElbow},
			{pose.leftElbow, pose.leftHand}, {pose.chest, pose.rightShoulder},
			{pose.rightShoulder, pose.rightElbow}, {pose.rightElbow, pose.rightHand},
			{pose.pelvis, pose.leftHip}, {pose.leftHip, pose.leftKnee},
			{pose.leftKnee, pose.leftFoot}, {pose.pelvis, pose.rightHip},
			{pose.rightHip, pose.rightKnee}, {pose.rightKnee, pose.rightFoot}
		};
		snapshot->entities.push_back(std::move(player));
	}
	return snapshot;
}

static int RunOverlayDemo(HINSTANCE instance)
{
	Overlay::Callbacks callbacks;
	callbacks.status = [] { return std::string("Demo - synthetic players"); };
	callbacks.draw = [] {
		const auto viewport = ImGui::GetIO().DisplaySize;
		RobRunReader::PublishDemoSnapshot(MakeDemoSnapshot(static_cast<float>(ImGui::GetTime())));
		RobRunReader::BeginRenderFrame(static_cast<int>(viewport.x), static_cast<int>(viewport.y));
		DrawPlayers();
		RobRunReader::EndRenderFrame();
		ImGui::GetForegroundDrawList()->AddText(ImVec2(20.f, 20.f), IM_COL32_WHITE,
			"Demo - synthetic players | Esc to close");
		if ((GetAsyncKeyState(VK_ESCAPE) & 0x8000) != 0) Overlay::RequestExit();
	};
	const int result = Overlay::Run(instance, L"VeoxDMA Demo", L"VeoxDMADemo", callbacks);
	RobRunReader::PublishDemoSnapshot({});
	return result;
}

static int RunSelfTests()
{
	int failures = 0;
	auto check = [&](bool ok, const char* name) {
		Diag("self-test %s: %s", ok ? "PASS" : "FAIL", name);
		if (!ok) ++failures;
	};
	try {
		ConfigInstances source;
		source.LootFilter.InitDefaults();
		auto& item = source.LootFilter.GetItemMut("perfume_3");
		item.customName = "Sample perfume";
		item.maxDistance = 120;
		item.useGlobalDistance = false;
		source.Player.MaxDistance = 310;
		source.Radar.Enabled = true;
		ConfigInstances loaded;
		loaded.FromJson(json::parse(source.ToJson().dump()));
		const auto& result = loaded.LootFilter.GetItem("perfume_3");
		check(loaded.Player.MaxDistance == 310 && loaded.Radar.Enabled &&
			result.customName == "Sample perfume" && result.maxDistance == 120 &&
			!result.useGlobalDistance, "configuration JSON round trip");

		const auto camera = DemoCamera();
		Matrix4x4 mat{};
		const bool built = RobRunCamera::BuildViewProjection(camera, 16.f / 9.f, mat);
		Vector2 point{};
		check(built && RobRunCamera::WorldToScreen(mat, {0.f, 1.6f, 10.f}, 1600, 900, point) &&
			std::abs(point.x - 800.f) < 1.f && std::abs(point.y - 450.f) < 1.f,
			"camera center projects to viewport center");
		check(built && !RobRunCamera::WorldToScreen(mat, {0.f, 1.6f, -10.f}, 1600, 900, point),
			"point behind camera is rejected");
		const auto demo = MakeDemoSnapshot(1.f);
		bool drawable = demo->entities.size() == 3;
		for (const auto& player : demo->entities)
			drawable = drawable && !player.bones.empty() &&
				RobRunCamera::WorldToScreen(mat, player.bonePoints.head, 1600, 900, point);
		check(built && drawable, "demo player poses project");

		RobRunWorld::State world;
		RobRunWorld::Entity bottle;
		bottle.kind = RobRunWorld::Kind::RandomLoot;
		bottle.label = "alcohol_9";
		world.entities.push_back(bottle);
		bottle.label = "alcohol_8";
		world.entities.push_back(bottle);
		const auto named = RobRunWorld::FormatLootDump(world, "omega");
		const auto indexed = RobRunWorld::FormatLootDump(world, "1");
		check(named.find("alcohol_9") != std::string::npos &&
			named.find("alcohol_8") == std::string::npos &&
			indexed.find("alcohol_8") != std::string::npos &&
			indexed.find("alcohol_9") == std::string::npos, "loot dump filters by name and index");
		RobRunWorld::State paintings;
		RobRunWorld::Entity picture;
		picture.kind = RobRunWorld::Kind::StaticLoot;
		picture.label = "trash_picture_5";
		picture.inventoryBasePrice = 1234;
		paintings.entities.push_back(picture);
		picture.label = "original_picture_5";
		picture.inventoryBasePrice = 5678;
		paintings.entities.push_back(picture);
		const auto exported = RobRunWorld::FormatLootDump(paintings, "Painting");
		check(exported.find("\"trash_picture_5\"\t\"Painting\"\ttrash_picture_5\tpicture_tube_1") != std::string::npos &&
			exported.find("\t1234\t\n") != std::string::npos &&
			exported.find("\t5678\t\n") != std::string::npos,
			"painting aliases keep their sprite and separate base prices");
	} catch (const std::exception& error) {
		Diag("self-test exception: %s", error.what());
		++failures;
	}
	Diag("self-test complete: %d failure(s)", failures);
	return failures ? 1 : 0;
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int)
{
	g_hInstance = hInstance;
	_set_se_translator(SeTranslator);
	AllocConsole();
	FILE* fDummy;
	freopen_s(&fDummy, "CONIN$", "r", stdin);
	freopen_s(&fDummy, "CONOUT$", "w", stderr);
	freopen_s(&fDummy, "CONOUT$", "w", stdout);
	SetConsoleCtrlHandler(ConsoleCtrlHandler, TRUE);
	printf("VeoxDMA - Starting...\n");
	Diag("=== VeoxDMA starting (pid %lu) ===", GetCurrentProcessId());
	DLOG("=== WinMain entry ===");

	int argCount = 0;
	LPWSTR* args = CommandLineToArgvW(GetCommandLineW(), &argCount);
	bool selfTest = false;
	bool demo = false;
	for (int i = 1; args && i < argCount; ++i) {
		const std::wstring_view arg(args[i]);
		selfTest |= arg == L"--selftest";
		demo |= arg == L"--demo";
	}
	if (args) LocalFree(args);
	if (selfTest || demo) {
		const int result = selfTest ? RunSelfTests() : RunOverlayDemo(hInstance);
		g_shutdownComplete.store(true, std::memory_order_release);
		return result;
	}

	TargetProcess.LoadRuntime();

	// Apply saved settings before starting background threads or drawing the menu.
	SetStage("config");
	ConfigFile::Load();
	Diag("config loaded");
	DLOG("Config loaded");

	StartAttachLoop();
	DLOG("AttachLoop started");

	// Auto-connect saved aim device so it's ready before the first frame.
	{
		using Base::Hardware::DeviceKind;
		constexpr DeviceKind deviceKinds[] = { DeviceKind::Off, DeviceKind::KmboxUsb, DeviceKind::KmboxNet,
			DeviceKind::Makcu, DeviceKind::Controller, DeviceKind::Ferrum, DeviceKind::Medius,
			DeviceKind::KmboxNbPro };
		constexpr int baudRates[] = { 115200, 921600, 1500000, 2000000, 3000000, 4000000 };
		const auto& saved = ConfigInstance.Aim;
		if (saved.DeviceIndex > 0 && static_cast<unsigned>(saved.DeviceIndex) < std::size(deviceKinds)
			&& static_cast<unsigned>(saved.BaudIndex) < std::size(baudRates)) {
			Base::Hardware::DeviceConfig cfg;
			cfg.kind = deviceKinds[saved.DeviceIndex];
			cfg.port = saved.ComPort;
			cfg.ip = saved.IpAddr;
			cfg.networkPort = saved.NetPort;
			cfg.mac = saved.MacId;
			cfg.baud = baudRates[saved.BaudIndex];
			cfg.manualPort = saved.ManualPort || cfg.kind == DeviceKind::Ferrum;
			cfg.ferrumController = saved.FerrumController;
			Base::Hardware::Session().Connect(cfg);
		}
	}

	Overlay::Callbacks callbacks;
	callbacks.init = [] {
		// Render thread: own SEH translator, then the init stages.
		_set_se_translator(SeTranslator);
		DLOG("Overlay init callback");
		SetStage("overlay:menu");
		Diag("overlay init: menu");
		VeoxMenu::Init();
		SetStage("overlay:radar");
		Radar::Init();
		WebRadar::Init();
		SetStage("overlay:weapons");
		WeaponSprites::Init();
		SetStage("overlay:icons");
		IconSprites::Init();
		Diag("overlay init: icons (dll=%d, embedded=%d)",
			IconSprites::UsingEmbedded() ? 1 : 0, IconSprites::EmbeddedCount());
		SetStage("overlay:backdrop");
		Backdrop::Create(g_hInstance);
		SetStage("overlay:aim");
		Aim::StartAimWorker();
		SetStage("overlay:ready");
		Diag("overlay init: done");
		DLOG("Overlay init callback done");
	};
	callbacks.tick = [] {
		TickOverlay();
	};
	callbacks.draw = [] {
		static int frameCount = 0;
		++frameCount;
		if (frameCount <= 3 || (frameCount % 500) == 0) {
			DLOG("Draw frame #%d", frameCount);
			SetStage("draw");
			if (frameCount == 1) Diag("first draw frame");
		}
		try {
			struct FrameCameraGuard {
				FrameCameraGuard(int w, int h) { RobRunReader::BeginRenderFrame(w, h); }
				~FrameCameraGuard() { RobRunReader::EndRenderFrame(); }
			};
			const ImVec2 display = ImGui::GetIO().DisplaySize;
			// Only draw game state while attached: during a reattach the reader
			// is being shut down and the last snapshot is stale.
			if (g_attached.load(std::memory_order_acquire)) {
				FrameCameraGuard cameraGuard((int)display.x, (int)display.y);
				const auto frameSnap = RobRunReader::GetSnapshot();
				const bool suppress3D = frameSnap &&
					frameSnap->inGameMap.valid &&
					frameSnap->inGameMap.fullscreenOpen &&
					ConfigInstance.Radar.InGameFullMap &&
					ConfigInstance.Radar.Hide3DEspOnFullMap;
				if (!suppress3D) {
					SetStage("draw:players");
					DrawPlayers();
					SetStage("draw:bvh");
					DrawBvhDebug();
					if (!ConfigInstance.Overlay.BattleMode) {
						SetStage("draw:world");
						DrawWorldEntities();
						SetStage("draw:house");
						DrawHouseInfo();
						SetStage("draw:radar");
						Radar::Draw();
						SetStage("draw:fullmap");
						Radar::DrawFullMap();
					}
					SetStage("draw:aim");
					Aim::Draw();
					Aim::DrawDebug();
				} else {
					RobRunReader::ReportRenderState(false, FLT_MAX);
				}
				SetStage("draw:ingamemap");
				Radar::DrawInGameMapOverlay();
			} else {
				SetStage("draw:aim");
				Aim::Draw();
				Aim::DrawDebug();
			}
			if (Overlay::MenuOpen()) {
				SetStage("draw:menu");
				VeoxMenu::Draw(g_attached.load(std::memory_order_acquire), GetAttachStatus());
			}
			SetStage("draw:idle");
		} catch (const std::exception& e) {
			Diag("draw exception frame %d at stage=%s: %s", frameCount, g_stage.load(), e.what());
			DLOG("Draw EXCEPTION frame #%d: %s", frameCount, e.what());
		} catch (...) {
			Diag("draw unknown exception frame %d at stage=%s", frameCount, g_stage.load());
			DLOG("Draw UNKNOWN EXCEPTION frame #%d", frameCount);
		}
	};
	callbacks.shutdown = [] {
		g_attached.store(false, std::memory_order_release);
		// Join the attach thread before stopping workers so a late attach
		// completion cannot restart the aim worker during teardown.
		StopAttachLoop();
		Aim::StopAimWorker();
		ConfigFile::Save();
		IconSprites::Shutdown();
		WeaponSprites::Shutdown();
		WebRadar::Stop();
		Radar::Shutdown();
		Backdrop::Destroy();
		{
			std::lock_guard<std::mutex> lifecycle(g_lifecycleMutex);
			SkinChanger::Stop();
			FovChanger::Reset();
			RobRunReader::Shutdown();
		}
	};
	callbacks.status = [] {
		return g_attached.load(std::memory_order_acquire) ? std::string("Attached") : GetAttachStatus();
	};

	const int result = Overlay::Run(hInstance, L"VeoxDMA", L"VeoxDMA", callbacks);
	// Run only invokes the shutdown callback on its normal teardown path. Join
	// here too so an early startup failure cannot leave a joinable thread for
	// the static destructor to terminate on.
	Aim::StopAimWorker();
	StopAttachLoop();
	SkinChanger::Stop();
	FovChanger::Reset();
	RobRunReader::Shutdown();
	g_shutdownComplete.store(true, std::memory_order_release);
	return result;
}
