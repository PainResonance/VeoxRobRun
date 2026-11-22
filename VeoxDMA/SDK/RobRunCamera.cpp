#include "pch.h"
#include "RobRunCamera.h"
#include "RobRunSDK.h"
#include "Il2Cpp.h"
#include "Memory.h"
#include "RobRunMemory.h"
#include "InputManager.h"
#include "SkeletonExtractor.h"
#include "RobRunReader.h"
#include <cstdio>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <mutex>

namespace RobRunCamera {

static std::atomic<std::uintptr_t> gaBase{ 0 };
static std::atomic<std::uintptr_t> g_cachedBrain{ 0 };
static std::mutex g_coreClassMutex;
static std::uintptr_t g_coreClass = 0;
static std::uint64_t g_coreClassAttemptMs = 0;

static std::uintptr_t ResolveCoreClass()
{
    std::lock_guard<std::mutex> lock(g_coreClassMutex);
    if (g_coreClass) return g_coreClass;
    const auto now = GetTickCount64();
    if (g_coreClassAttemptMs && now - g_coreClassAttemptMs < 1000) return 0;
    g_coreClassAttemptMs = now;
    const auto indexed = Il2Cpp::GetClassByIndex(rr::ClassIndex::CinemachineCore);
    if (Il2Cpp::ClassNameOfClass(indexed) == "CinemachineCore")
        g_coreClass = indexed;
    else
        g_coreClass = Il2Cpp::FindClassByName("CinemachineCore");
    return g_coreClass;
}

struct ManagedPhysCache {
    std::uintptr_t va = 0;
    std::uint64_t pa = 0;
    std::uint64_t stampMs = 0;
};
static std::mutex g_managedPhysMutex;
static ManagedPhysCache g_managedPhys{};
static float g_cachedFov = 0.f;
static std::uint64_t g_cachedFovStampMs = 0;

static bool ReadBytes(std::uintptr_t address, void* buffer, std::size_t size)
{
    if (!address || !buffer || !size) return false;
    return TargetProcess.Read(address, buffer, size);
}

static bool ReadBytesFresh(std::uintptr_t address, void* buffer, std::size_t size)
{
    if (!address || !buffer || !size) return false;
    return TargetProcess.ReadFresh(address, buffer, size);
}

static std::uintptr_t ReadPtr(std::uintptr_t address)
{
    std::uintptr_t value = 0;
    return ReadBytes(address, &value, sizeof(value)) ? value : 0;
}

static bool Finite(const Vector3& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

static bool Normalize(Vector3& v)
{
    const float lengthSq = v.x * v.x + v.y * v.y + v.z * v.z;
    if (!std::isfinite(lengthSq) || lengthSq < 1e-8f) return false;
    const float inv = 1.f / std::sqrt(lengthSq);
    v.x *= inv; v.y *= inv; v.z *= inv;
    return Finite(v);
}

static Vector3 Cross(const Vector3& a, const Vector3& b)
{
    return Vector3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x);
}

static Vector3 Rotate(const float q[4], const Vector3& v)
{
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    const Vector3 t{
        y * v.z - z * v.y,
        z * v.x - x * v.z,
        x * v.y - y * v.x
    };
    const Vector3 t2{ t.x * 2.f, t.y * 2.f, t.z * 2.f };
    const Vector3 cross{
        y * t2.z - z * t2.y,
        z * t2.x - x * t2.z,
        x * t2.y - y * t2.x
    };
    return Vector3(
        v.x + w * t2.x + cross.x,
        v.y + w * t2.y + cross.y,
        v.z + w * t2.z + cross.z);
}

void Init(std::uintptr_t gameAssemblyBase)
{
    gaBase.store(gameAssemblyBase, std::memory_order_release);
    g_cachedBrain.store(0, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(g_coreClassMutex);
        g_coreClass = 0;
        g_coreClassAttemptMs = 0;
    }
    g_cachedFov = 0.f;
    g_cachedFovStampMs = 0;
    {
        std::lock_guard<std::mutex> lock(g_managedPhysMutex);
        g_managedPhys = ManagedPhysCache{};
    }
    Il2Cpp::Init(gameAssemblyBase);
}

float ReadCinemachineFov()
{
    if (!gaBase.load(std::memory_order_acquire)) return 0.f;

    // The brain instance is stable for a scene, so it is resolved once and
    // then only the FOV value is re-read (uncached) each pass.
    const std::uintptr_t cached = g_cachedBrain.load(std::memory_order_acquire);
    if (cached > 0x10000) {
        float fov = 0.f;
        if (ReadBytesFresh(cached + rr::Cinemachine::BrainFov, &fov, sizeof(fov))) {
            return std::isfinite(fov) && fov >= 1.f && fov <= 179.f ? fov : 0.f;
        }
        g_cachedBrain.store(0, std::memory_order_release);
    }

    const std::uintptr_t klass = ResolveCoreClass();
    if (!klass) return 0.f;

    const std::uintptr_t statics = Il2Cpp::StaticFields(klass);
    if (!statics) return 0.f;

    const std::uintptr_t core = ReadPtr(statics + rr::Cinemachine::CoreInstance);
    if (!core) return 0.f;

    const std::uintptr_t brainList = ReadPtr(core + rr::Cinemachine::BrainList);
    if (!brainList) return 0.f;

    const std::uintptr_t items = ReadPtr(brainList + rr::List::items);
    std::int32_t count = 0;
    if (!items || !ReadBytes(brainList + rr::List::count, &count, sizeof(count)))
        return 0.f;
    if (count < 1 || count > 32) return 0.f;

    for (int i = 0; i < count; ++i) {
        const std::uintptr_t brain = ReadPtr(items + rr::Array::vector +
            static_cast<std::uintptr_t>(i) * rr::Array::stride);
        if (!brain || !Il2Cpp::IsClass(brain, "CinemachineBrain")) continue;
        // The brain is useful even before its lens state has a valid FOV.
        g_cachedBrain.store(brain, std::memory_order_release);
        float fov = 0.f;
        if (ReadBytesFresh(brain + rr::Cinemachine::BrainFov, &fov, sizeof(fov)) &&
            std::isfinite(fov) && fov >= 1.f && fov <= 179.f) {
            return fov;
        }
        return 0.f;
    }
    return 0.f;
}

// RRX's scanner anchors BattleAimTargetUpdater._battleEntitiesModel (+0x30)
// to the active model, then follows _cinemachineBrain (+0x48). The owning
// BattleUpdater exposes this updater directly, avoiding a whole-heap scan.
static std::uintptr_t ReadAimUpdaterBrain(std::uintptr_t updater, std::uintptr_t entities)
{
    if (!Il2Cpp::IsClass(updater, "BattleAimTargetUpdater")) return 0;
    if (entities && ReadPtr(updater + 0x30) != entities) return 0;
    const auto brain = ReadPtr(updater + rr::Fov::AimBrain);
    return Il2Cpp::IsClass(brain, "CinemachineBrain") ? brain : 0;
}

static std::uintptr_t ReadBattleUpdaterBrain(std::uintptr_t game, std::uintptr_t views)
{
    static std::atomic<std::uint64_t> lastDiagnostic{0};
    const auto now = GetTickCount64();
    auto previous = lastDiagnostic.load(std::memory_order_relaxed);
    const bool diagnostic = now - previous >= 5000 &&
        lastDiagnostic.compare_exchange_strong(previous, now, std::memory_order_relaxed);
    const auto owner = RobRunReader::FindActiveUpdaterByName(game, "BattleUpdater");
    if (!owner) {
        if (diagnostic)
            std::printf("[CameraBrain] BattleUpdater not found: game=0x%llX class='%s' views=0x%llX class='%s'\n",
                static_cast<unsigned long long>(game), Il2Cpp::ClassNameOf(game).c_str(),
                static_cast<unsigned long long>(views), Il2Cpp::ClassNameOf(views).c_str());
        return 0;
    }
    const auto entities = Il2Cpp::IsClass(views, "BattleEntitiesUpdater")
        ? ReadPtr(views + 0x18) : 0;
    if (!entities || !Il2Cpp::IsClass(entities, "BattleEntitiesModel")) {
        if (diagnostic)
            std::printf("[CameraBrain] Invalid entity anchor: owner=0x%llX views=0x%llX entities=0x%llX class='%s'\n",
                static_cast<unsigned long long>(owner), static_cast<unsigned long long>(views),
                static_cast<unsigned long long>(entities), Il2Cpp::ClassNameOf(entities).c_str());
        return 0;
    }

    Il2Cpp::Field field{};
    const auto klass = Il2Cpp::ClassOf(owner);
    const bool named = Il2Cpp::FindField(klass, "_battleAimTargetUpdater", field);
    const auto offset = named && field.offset >= 0x10
        ? static_cast<std::uintptr_t>(field.offset) : 0x1C0;
    const auto updater = ReadPtr(owner + offset);
    const auto brain = ReadAimUpdaterBrain(updater, entities);
    if (!brain) {
        if (diagnostic) {
            std::printf("[CameraBrain] owner=0x%llX aimOffset=0x%llX aim=0x%llX class='%s' expectedEntities=0x%llX aimEntities=0x%llX brain=0x%llX\n",
                static_cast<unsigned long long>(owner), static_cast<unsigned long long>(offset),
                static_cast<unsigned long long>(updater), Il2Cpp::ClassNameOf(updater).c_str(),
                static_cast<unsigned long long>(entities), static_cast<unsigned long long>(ReadPtr(updater+0x30)),
                static_cast<unsigned long long>(ReadPtr(updater+rr::Fov::AimBrain)));
        }
    }
    return brain;
}

std::uintptr_t GetCachedBrain(std::uintptr_t views, std::uintptr_t game)
{
    const auto cached = g_cachedBrain.load(std::memory_order_acquire);
    if (cached > 0x10000 && Il2Cpp::IsClass(cached, "CinemachineBrain")) return cached;
    g_cachedBrain.store(0, std::memory_order_release);

    // Global camera ownership lives on BattleUpdater, outside entity dictionaries.
    if (game) {
        const auto brain = ReadBattleUpdaterBrain(game, views);
        if (brain) {
            g_cachedBrain.store(brain, std::memory_order_release);
            return brain;
        }
    }

    // This is the same live-updater route used by the FOV changer. Prefer it
    // when supplied, so Freecam does not depend on class-table numeric ids.
    if (views && Il2Cpp::IsClass(views, "BattleEntitiesUpdater")) {
        const std::uintptr_t offsets[] = {rr::Fov::BeuUpdaters, rr::Fov::BeuViewUpdaters};
        for (const auto offset : offsets) {
            std::vector<RobRunMemory::DictEntry> entries;
            if (!RobRunMemory::ReadDictInt(ReadPtr(views + offset), entries, false)) continue;
            for (const auto& entry : entries) {
                if (!Il2Cpp::IsClass(entry.value, "BattleAimTargetUpdater")) continue;
                const auto brain = ReadPtr(entry.value + rr::Fov::AimBrain);
                if (!brain || !Il2Cpp::IsClass(brain, "CinemachineBrain")) continue;
                g_cachedBrain.store(brain, std::memory_order_release);
                return brain;
            }
        }
    }
    ReadCinemachineFov();
    return g_cachedBrain.load(std::memory_order_acquire);
}

namespace {
struct LocalPose {
    float position[4];
    float rotation[4];
    float scale[4];
};
static_assert(sizeof(LocalPose) == 48);
struct CameraPipeline {
    std::uintptr_t updater = 0;
    std::uintptr_t composite = 0;
    std::uintptr_t compositeField = 0;
    std::uintptr_t listField = 0;
    std::uintptr_t list = 0;
    std::uintptr_t items = 0;
    std::int32_t count = 0;
    std::int32_t version = 0;
    std::vector<std::uintptr_t> entries;
    bool paused = false;
};
struct FreeCamera {
    std::uintptr_t brain = 0, hierarchy = 0, matrices = 0, parents = 0, owners = 0, transform = 0;
    int index = -1, count = 0, updateMode = 0;
    std::uintptr_t countAddress = 0;
    std::uint64_t session = 0, stamp = 0;
    LocalPose saved{};
    CameraPipeline pipeline;
    Vector3 position{};
    float yaw = 0, pitch = 0;
    bool active = false;
} freeCamera;
bool keyF3 = false, keySlower = false, keyFaster = false;
std::mutex freecamPoseMutex;
CameraState freecamPose;
std::atomic<const char*> freecamStatus{"Freecam off"};

void SetFreecamStatus(const char* status)
{
    const char* previous = freecamStatus.exchange(status, std::memory_order_relaxed);
    if (std::strcmp(previous, status) != 0)
        std::printf("[Freecam] %s\n", status);
}

bool ReadPose(std::uintptr_t address, LocalPose& pose)
{
    if (!ReadBytesFresh(address, &pose, sizeof(pose))) return false;
    float rotationLength = 0.f;
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(pose.rotation[i])) return false;
        rotationLength += pose.rotation[i] * pose.rotation[i];
    }
    if (rotationLength < 0.9f || rotationLength > 1.1f) return false;
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(pose.position[i]) || !std::isfinite(pose.scale[i]) ||
            std::fabs(pose.scale[i]) < 0.0001f) return false;
    }
    return true;
}

std::uintptr_t ReferenceFieldAddress(std::uintptr_t object, const char* name, std::uintptr_t fallback)
{
    Il2Cpp::Field field{};
    const bool found = Il2Cpp::FindField(Il2Cpp::ClassOf(object), name, field);
    const auto offset = found && field.offset >= 0x10
        ? static_cast<std::uintptr_t>(field.offset) : fallback;
    return object + offset;
}

std::uintptr_t ReferenceField(std::uintptr_t object, const char* name, std::uintptr_t fallback)
{
    std::uintptr_t value = 0;
    return ReadBytesFresh(ReferenceFieldAddress(object, name, fallback), &value, sizeof(value)) ? value : 0;
}

struct CameraListHeader {
    std::uintptr_t items = 0;
    std::int32_t count = 0;
    std::int32_t version = 0;
};
static_assert(sizeof(CameraListHeader) == 16);

bool ReadCameraList(std::uintptr_t list, CameraListHeader& header,
    std::vector<std::uintptr_t>& entries)
{
    if (!ReadBytesFresh(list + rr::List::items, &header, sizeof(header)) ||
        header.items < 0x10000 || header.count < 0 || header.count > 128) return false;
    std::uint64_t capacity = 0;
    if (!ReadBytesFresh(header.items + rr::Array::max_length, &capacity, sizeof(capacity)) ||
        capacity < static_cast<std::uint64_t>(header.count) || capacity > 65536) return false;
    entries.resize(header.count);
    if (!entries.empty() && !ReadBytesFresh(header.items + rr::Array::vector,
        entries.data(), entries.size() * sizeof(std::uintptr_t))) return false;
    CameraListHeader after{};
    return ReadBytesFresh(list + rr::List::items, &after, sizeof(after)) &&
        header.items == after.items && header.count == after.count && header.version == after.version;
}

bool ContainsCameraBrain(std::uintptr_t updater, std::uintptr_t brain, int depth)
{
    if (Il2Cpp::IsClass(updater, "CameraBrainUpdater"))
        return ReferenceField(updater, "_cameraBrain", 0x10) == brain;
    if (depth >= 4 || !Il2Cpp::IsClass(updater, "CompositeUpdater")) return false;
    CameraListHeader header{};
    std::vector<std::uintptr_t> children;
    if (!ReadCameraList(ReferenceField(updater, "_updaters", 0x10), header, children)) return false;
    for (const auto child : children)
        if (ContainsCameraBrain(child, brain, depth + 1)) return true;
    return false;
}

bool ResolveCameraPipeline(std::uintptr_t game, std::uintptr_t brain, CameraPipeline& pipeline)
{
    const auto owner = RobRunReader::FindActiveUpdaterByName(game, "BattleUpdater");
    if (!owner) return false;
    const auto updater = ReferenceField(owner, "_battleCameraUpdater", 0x128);
    if (!Il2Cpp::IsClass(updater, "BattleCameraUpdater")) return false;
    const auto composite = ReferenceField(updater, "_updater", 0x10);
    if (!Il2Cpp::IsClass(composite, "CompositeUpdater")) {
        std::printf("[Freecam] Unsupported camera pipeline class '%s'\n",
            Il2Cpp::ClassNameOf(composite).c_str());
        return false;
    }
    const auto list = ReferenceField(composite, "_updaters", 0x10);
    CameraListHeader header{};
    if (!ReadCameraList(list, header, pipeline.entries) || header.count == 0 ||
        !ContainsCameraBrain(composite, brain, 0)) return false;
    pipeline.updater = updater;
    pipeline.composite = composite;
    pipeline.compositeField = ReferenceFieldAddress(updater, "_updater", 0x10);
    pipeline.listField = ReferenceFieldAddress(composite, "_updaters", 0x10);
    pipeline.list = list;
    pipeline.items = header.items;
    pipeline.count = header.count;
    pipeline.version = header.version;
    return true;
}

bool OwnsCameraPipeline(const CameraPipeline& pipeline)
{
    std::uintptr_t composite = 0, list = 0;
    return ReadBytesFresh(pipeline.compositeField, &composite, sizeof(composite)) &&
        composite == pipeline.composite &&
        ReadBytesFresh(pipeline.listField, &list, sizeof(list)) && list == pipeline.list;
}

bool PauseCameraPipeline(CameraPipeline& pipeline)
{
    CameraListHeader header{};
    std::vector<std::uintptr_t> entries;
    if (!OwnsCameraPipeline(pipeline) || !ReadCameraList(pipeline.list, header, entries) ||
        header.items != pipeline.items || header.count != pipeline.count ||
        header.version != pipeline.version || entries != pipeline.entries) return false;
    // The backing array retains all references. One count write pauses the
    // private camera composite without shifting entries or changing a version
    // that an in-progress List enumerator may be checking.
    pipeline.paused = true;
    const std::int32_t zero = 0;
    std::int32_t count = -1;
    return TargetProcess.Write(pipeline.list + rr::List::count, zero) &&
        ReadBytesFresh(pipeline.list + rr::List::count, &count, sizeof(count)) && count == 0;
}

bool RestoreCameraPipeline(CameraPipeline& pipeline)
{
    if (!pipeline.paused) return true;
    // A recreated camera owns a different collection. Leave that collection
    // under the game's control instead of restoring a stale count into it.
    std::uintptr_t composite = 0, list = 0;
    if (!ReadBytesFresh(pipeline.compositeField, &composite, sizeof(composite))) return false;
    if (composite != pipeline.composite) { pipeline.paused = false; return true; }
    if (!ReadBytesFresh(pipeline.listField, &list, sizeof(list))) return false;
    if (list != pipeline.list) { pipeline.paused = false; return true; }
    CameraListHeader header{};
    if (!ReadBytesFresh(pipeline.list + rr::List::items, &header, sizeof(header))) return false;
    if (header.items != pipeline.items || header.version != pipeline.version ||
        (header.count != 0 && header.count != pipeline.count)) return false;
    std::vector<std::uintptr_t> entries(pipeline.entries.size());
    if (!ReadBytesFresh(header.items + rr::Array::vector, entries.data(),
        entries.size() * sizeof(std::uintptr_t)) || entries != pipeline.entries) return false;
    if (!TargetProcess.Write(pipeline.list + rr::List::count, pipeline.count)) return false;
    std::int32_t count = 0;
    if (!ReadBytesFresh(pipeline.list + rr::List::count, &count, sizeof(count)) ||
        count != pipeline.count) return false;
    pipeline.paused = false;
    return true;
}

bool CameraPipelinePaused(const CameraPipeline& pipeline)
{
    CameraListHeader header{};
    return pipeline.paused && OwnsCameraPipeline(pipeline) &&
        ReadBytesFresh(pipeline.list + rr::List::items, &header, sizeof(header)) &&
        header.items == pipeline.items && header.count == 0 && header.version == pipeline.version;
}

bool Down(int key)
{
    return Keyboard::IsReady() && Keyboard::IsKeyDown(key);
}
bool Edge(int key, bool& previous)
{
    const bool down = Down(key);
    const bool pressed = down && !previous;
    previous = down;
    return pressed;
}
Vector3 InverseRotate(const float q[4], const Vector3& v)
{
    const float inverse[] = {-q[0], -q[1], -q[2], q[3]};
    return Rotate(inverse, v);
}
void Multiply(const float a[4], const float b[4], float out[4])
{
    const float result[] = {
        a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
        a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
        a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
        a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]};
    std::memcpy(out, result, sizeof(result));
}
// Compute the current parent pose, so movement stays in world space even
// while the player's camera rig moves. Bound and validate every hierarchy walk.
bool WorldPose(const FreeCamera& camera, int index, LocalPose& world)
{
    if (index < 0 || index >= camera.count) return false;
    if (!ReadPose(camera.matrices + 48ull*index, world)) return false;
    for (int depth = 0; depth < camera.count; ++depth) {
        int parent = -1;
        if (!ReadBytesFresh(camera.parents + 4ull*index, &parent, sizeof(parent))) return false;
        if (parent == -1) return RobRunMemory::Finite(Vector3(world.position[0],world.position[1],world.position[2]));
        if (parent < 0 || parent >= camera.count) return false;
        LocalPose pose{};
        if (!ReadPose(camera.matrices + 48ull*parent, pose)) return false;
        const Vector3 p = Rotate(pose.rotation, Vector3(world.position[0]*pose.scale[0],
            world.position[1]*pose.scale[1], world.position[2]*pose.scale[2]));
        world.position[0] = p.x + pose.position[0];
        world.position[1] = p.y + pose.position[1];
        world.position[2] = p.z + pose.position[2];
        Multiply(pose.rotation,world.rotation,world.rotation);
        for (int i=0;i<3;++i) world.scale[i] *= pose.scale[i];
        index = parent;
    }
    return false;
}
bool ValidCamera(const FreeCamera& camera)
{
    return camera.brain && Il2Cpp::IsClass(camera.brain,"CinemachineBrain") &&
        RobRunMemory::ReadPtr(camera.transform+0x28) == camera.hierarchy &&
        RobRunMemory::ReadInt32(camera.countAddress) == camera.count &&
        RobRunMemory::ReadPtr(camera.hierarchy+0x18) == camera.matrices &&
        RobRunMemory::ReadPtr(camera.hierarchy+0x20) == camera.parents &&
        RobRunMemory::ReadPtr(camera.hierarchy+0x30) == camera.owners &&
        RobRunMemory::ReadPtr(camera.owners+8ull*camera.index) == camera.transform;
}
bool ResolveFreeCamera(const CameraState& eye, std::uintptr_t views, std::uintptr_t game, FreeCamera& result)
{
    const auto brain = GetCachedBrain(views, game);
    if (!brain || !Il2Cpp::IsClass(brain,"CinemachineBrain")) { SetFreecamStatus("CinemachineBrain is unavailable"); return false; }
    const auto gameObject = RobRunMemory::ReadPtr(RobRunMemory::ReadPtr(brain+0x50)+0x10);
    if (gameObject < 0x10000) { SetFreecamStatus("Output camera is unavailable"); return false; }
    float best = 2.25f;
    for (int offset=8;offset<=0x100;offset+=8) {
        const auto object = RobRunMemory::ReadPtr(gameObject+offset);
        const int count = RobRunMemory::ReadInt32(object+0x30);
        const auto components = RobRunMemory::ReadPtr(object+0x20);
        if (count<1 || count>512 || components<0x10000) continue;
        for (int component=0;component<std::min(count * 2,128);++component) {
            FreeCamera candidate;
            candidate.brain = brain;
            candidate.transform = RobRunMemory::ReadPtr(components+8ull*component);
            Skeleton::TransformTarget target;
            if (!Skeleton::ResolveNativeTransform(candidate.transform, ReadBytesFresh, target)) continue;
            candidate.hierarchy = target.hierarchy;
            candidate.index = target.index;
            candidate.count = target.count;
            candidate.countAddress = target.countAddress;
            candidate.matrices = target.matrices;
            candidate.parents = target.parents;
            candidate.owners = target.owners;
            LocalPose world{};
            if (!ValidCamera(candidate) || !WorldPose(candidate,candidate.index,world)) continue;
            const float dx=world.position[0]-eye.position.x, dy=world.position[1]-eye.position.y, dz=world.position[2]-eye.position.z;
            const float distance=dx*dx+dy*dy+dz*dz;
            if (distance>=best) continue;
            if (!ReadPose(candidate.matrices+48ull*candidate.index,candidate.saved)) continue;
            best=distance;
            result=candidate;
        }
    }
    if (!result.brain) SetFreecamStatus("No verified camera transform found");
    return result.brain != 0;
}
}

const char* GetFreecamStatus()
{
    return freecamStatus.load(std::memory_order_relaxed);
}

void StopFreecam(const char* reason)
{
    if (freeCamera.active) {
        bool restored = true;
        if (ValidCamera(freeCamera))
            restored = TargetProcess.Write(freeCamera.matrices+48ull*freeCamera.index,&freeCamera.saved,sizeof(LocalPose));
        if (Il2Cpp::IsClass(freeCamera.brain,"CinemachineBrain"))
            restored = TargetProcess.Write(freeCamera.brain+0x30,freeCamera.updateMode) && restored;
        restored = RestoreCameraPipeline(freeCamera.pipeline) && restored;
        // Keep ownership until restoration succeeds, allowing the next worker
        // pass to recover from a transient DMA write failure.
        if (!restored) { SetFreecamStatus("Camera restoration failed, retrying"); return; }
        SetFreecamStatus("Freecam off");
    }
    freeCamera = {};
    std::lock_guard<std::mutex> lock(freecamPoseMutex);
    freecamPose = {};
    if (reason) SetFreecamStatus(reason);
}

void TickFreecam(bool& enabled, float& speed, std::uint64_t session, const CameraState& eye, std::uintptr_t views, std::uintptr_t game)
{
    if (Edge(VK_F3,keyF3)) enabled = !enabled;
    const bool slower = Edge(VK_OEM_4,keySlower);
    const bool faster = Edge(VK_OEM_6,keyFaster);
    if (!enabled) { StopFreecam(); return; }
    if (freeCamera.active && freeCamera.session != session) {
        StopFreecam();
        SetFreecamStatus("Camera session changed");
        enabled = false;
        return;
    }
    if (!eye.valid) { StopFreecam(); SetFreecamStatus("Player camera is unavailable"); enabled=false; return; }
    if (!freeCamera.active) {
        FreeCamera resolved;
        if (!ResolveFreeCamera(eye,views,game,resolved)) { enabled=false; return; }
        if (!ReadBytesFresh(resolved.brain+0x30,&resolved.updateMode,sizeof(int)) ||
            resolved.updateMode < 0 || resolved.updateMode > 3) {
            SetFreecamStatus("Cannot read Cinemachine update mode");
            enabled = false;
            return;
        }
        if (!ResolveCameraPipeline(game, resolved.brain, resolved.pipeline)) {
            SetFreecamStatus("Cannot verify the game camera updater collection");
            enabled = false;
            return;
        }
        resolved.position=eye.position;
        resolved.yaw=std::atan2(eye.forward.x,eye.forward.z);
        resolved.pitch=std::asin(std::clamp(-eye.forward.y,-1.f,1.f));
        resolved.session=session;
        resolved.stamp=GetTickCount64();
        resolved.active=true;
        freeCamera = std::move(resolved);
        if (!PauseCameraPipeline(freeCamera.pipeline) ||
            !TargetProcess.Write(freeCamera.brain + 0x30, 3)) {
            StopFreecam("Cannot pause the game camera updater");
            enabled = false;
            return;
        }
        std::printf("[Freecam] Paused camera pipeline 0x%llX (%d updaters, previous brain mode=%d)\n",
            static_cast<unsigned long long>(freeCamera.pipeline.list),
            freeCamera.pipeline.count, freeCamera.updateMode);
        SetFreecamStatus("Freecam active");
    }
    auto& camera=freeCamera;
    const auto now=GetTickCount64();
    speed=std::clamp(std::isfinite(speed) ? speed+0.5f*(int(faster)-int(slower)) : 4.f,0.5f,60.f);
    // Hotkeys remain responsive at worker rate; transform transactions run
    // at camera rate to avoid saturating the DMA link.
    if (now-camera.stamp < 16) return;
    if (!ValidCamera(camera)) { StopFreecam(); SetFreecamStatus("Camera transform ownership changed"); enabled=false; return; }
    if (!CameraPipelinePaused(camera.pipeline)) {
        StopFreecam("Game camera updater collection changed");
        enabled = false;
        return;
    }
    const float dt=std::min(float(now-camera.stamp)*0.001f,0.064f);
    camera.stamp=now;
    const float rate=Down(VK_SHIFT)||Down(VK_LSHIFT)||Down(VK_RSHIFT) ? 3.f : 1.f;
    camera.yaw+=(int(Down('9'))-int(Down('7')))*dt*1.8f*rate;
    camera.pitch=std::clamp(camera.pitch+(int(Down('0'))-int(Down('8')))*dt*1.8f*rate,-1.55f,1.55f);
    const float sy=std::sin(camera.yaw*0.5f), cy=std::cos(camera.yaw*0.5f);
    const float sp=std::sin(camera.pitch*0.5f), cp=std::cos(camera.pitch*0.5f);
    float rotation[]={cy*sp,sy*cp,-sy*sp,cy*cp};
    float worldRotation[4];
    std::memcpy(worldRotation, rotation, sizeof(rotation));
    const auto forward=Rotate(rotation,Vector3(0,0,1));
    const auto right=Rotate(rotation,Vector3(1,0,0));
    const float f=float(int(Down('W'))-int(Down('S')));
    const float r=float(int(Down('D'))-int(Down('A')));
    const float u=float(int(Down(VK_PRIOR))-int(Down(VK_NEXT)));
    const float step=dt*speed*rate;
    camera.position.x+=(forward.x*f+right.x*r)*step;
    camera.position.y+=(forward.y*f+right.y*r+u)*step;
    camera.position.z+=(forward.z*f+right.z*r)*step;
    LocalPose pose=camera.saved;
    Vector3 position=camera.position;
    int parent = -1;
    if (!ReadBytesFresh(camera.parents+4ull*camera.index, &parent, sizeof(parent)) ||
        parent < -1 || parent >= camera.count) {
        StopFreecam("Camera parent table is unavailable");
        enabled = false;
        return;
    }
    if (parent>=0) {
        LocalPose world{};
        if (!WorldPose(camera,parent,world)) { StopFreecam("Cannot read camera parent pose"); enabled=false; return; }
        position=InverseRotate(world.rotation,Vector3(position.x-world.position[0],position.y-world.position[1],position.z-world.position[2]));
        for (int i=0;i<3;++i) {
            if (!std::isfinite(world.scale[i]) || std::fabs(world.scale[i])<0.0001f) { StopFreecam("Camera parent scale is invalid"); enabled=false; return; }
        }
        position.x/=world.scale[0]; position.y/=world.scale[1]; position.z/=world.scale[2];
        const float inverse[]={-world.rotation[0],-world.rotation[1],-world.rotation[2],world.rotation[3]};
        Multiply(inverse,rotation,rotation);
    }
    pose.position[0]=position.x; pose.position[1]=position.y; pose.position[2]=position.z;
    std::memcpy(pose.rotation,rotation,sizeof(rotation));
    if (!TargetProcess.Write(camera.matrices+48ull*camera.index,&pose,sizeof(pose))) {
        StopFreecam("Camera transform write failed");
        enabled = false;
        return;
    }
    // Publish the same world pose to managed-camera consumers, including
    // projection when Native ESP Camera is disabled.
    CameraState published = eye;
    published.position = camera.position;
    std::memcpy(published.orientation, worldRotation, sizeof(worldRotation));
    std::memcpy(published.look, worldRotation, sizeof(worldRotation));
    published.forward = forward;
    published.right = right;
    published.up = Rotate(worldRotation, Vector3(0,1,0));
    published.distance = 0.f;
    std::lock_guard<std::mutex> lock(freecamPoseMutex);
    freecamPose = published;
}

CameraState Read(std::uintptr_t battleCameraModel)
{
    CameraState camera{};
    if (battleCameraModel <= 0x10000) return camera;

    // The whole camera block (position, both quaternions, basis vectors and
    // distance) is contiguous. One uncached read keeps the projection aligned
    // with the frame the game is rendering, instead of a cached sample that
    // can trail by hundreds of milliseconds.
    struct CameraBlock {
        float position[3];    // +0x10
        float orientation[4]; // +0x1C
        float look[4];        // +0x2C
        float forward[3];     // +0x3C
        float right[3];       // +0x48
        float up[3];          // +0x54
        float distance;       // +0x60
    };
    static_assert(sizeof(CameraBlock) == 0x54, "camera block layout");
    static_assert(offsetof(CameraBlock, orientation) == 0x0C, "orientation offset");
    static_assert(offsetof(CameraBlock, look) == 0x1C, "look offset");
    static_assert(offsetof(CameraBlock, forward) == 0x2C, "forward offset");
    static_assert(offsetof(CameraBlock, right) == 0x38, "right offset");
    static_assert(offsetof(CameraBlock, up) == 0x44, "up offset");
    static_assert(offsetof(CameraBlock, distance) == 0x50, "distance offset");

    CameraBlock block{};
    const std::uintptr_t blockVa = battleCameraModel + rr::BattleCameraModelOffsets::Position;
    const bool singlePage = ((blockVa & ~static_cast<std::uintptr_t>(0xFFF)) ==
        ((blockVa + sizeof(block) - 1) & ~static_cast<std::uintptr_t>(0xFFF)));
    bool readOk = false;
    bool usedPhys = false;

    if (singlePage) {
        std::uint64_t pa = 0;
        const std::uint64_t nowMs = GetTickCount64();
        {
            std::lock_guard<std::mutex> lock(g_managedPhysMutex);
            if (g_managedPhys.va == blockVa && g_managedPhys.pa != 0 &&
                (nowMs - g_managedPhys.stampMs) < 3000) {
                pa = g_managedPhys.pa;
            }
        }
        if (!pa) {
            std::uint64_t translated = 0;
            if (TargetProcess.VirtToPhys(blockVa, translated) && translated != 0) {
                pa = translated;
                std::lock_guard<std::mutex> lock(g_managedPhysMutex);
                g_managedPhys = ManagedPhysCache{ blockVa, pa, nowMs };
            }
        }
        if (pa && TargetProcess.ReadPhysicalFast(pa, &block, sizeof(block))) {
            const Vector3 p(block.position[0], block.position[1], block.position[2]);
            const Vector3 f(block.forward[0], block.forward[1], block.forward[2]);
            const float fLenSq = f.x * f.x + f.y * f.y + f.z * f.z;
            if (Finite(p) && std::fabs(p.x) < 1e6f && std::fabs(p.y) < 1e6f && std::fabs(p.z) < 1e6f &&
                Finite(f) && fLenSq > 0.25f && fLenSq < 2.0f) {
                readOk = true;
                usedPhys = true;
            } else {
                std::lock_guard<std::mutex> lock(g_managedPhysMutex);
                g_managedPhys = ManagedPhysCache{};
            }
        }
    }

    if (!readOk &&
        !ReadBytesFresh(blockVa, &block, sizeof(block)) &&
        !ReadBytes(blockVa, &block, sizeof(block))) {
        return camera;
    }

    camera.usedPhysical = usedPhys;
    camera.position = Vector3(block.position[0], block.position[1], block.position[2]);
    if (!Finite(camera.position)) return camera;
    std::memcpy(camera.orientation, block.orientation, sizeof(camera.orientation));
    std::memcpy(camera.look, block.look, sizeof(camera.look));
    camera.forward = Vector3(block.forward[0], block.forward[1], block.forward[2]);
    camera.right = Vector3(block.right[0], block.right[1], block.right[2]);
    camera.up = Vector3(block.up[0], block.up[1], block.up[2]);
    if (std::isfinite(block.distance)) camera.distance = block.distance;

    const bool hasForward = Finite(camera.forward);
    const bool hasRight = Finite(camera.right);
    const bool hasUp = Finite(camera.up);

    if (!hasForward || !Normalize(camera.forward))
        camera.forward = Rotate(camera.look, Vector3(0.f, 0.f, 1.f));
    if (!Normalize(camera.forward)) return camera;

    const bool rightOk = hasRight && Normalize(camera.right);
    const bool upOk = hasUp && Normalize(camera.up);
    if (!rightOk || !upOk) {
        // Rebuild an orthonormal basis around forward. Unity is left-handed
        // with +Y up, so right = up x forward and up = forward x right.
        const Vector3 upHint = upOk ? camera.up : Vector3(0.f, 1.f, 0.f);
        Vector3 right = Cross(upHint, camera.forward);
        if (!Normalize(right)) return camera;
        Vector3 up = Cross(camera.forward, right);
        if (!Normalize(up)) return camera;
        camera.right = right;
        camera.up = up;
    }

    // Throttle Cinemachine FOV reads to 40 Hz (25 ms) so high-rate camera
    // polling doesn't burn a second uncached DMA transaction every cycle.
    const std::uint64_t fovNowMs = GetTickCount64();
    float fov = g_cachedFov;
    if (fov <= 0.f || (fovNowMs - g_cachedFovStampMs) >= 25) {
        const float freshFov = ReadCinemachineFov();
        if (freshFov > 0.f) {
            fov = freshFov;
            g_cachedFov = freshFov;
            g_cachedFovStampMs = fovNowMs;
        }
    }
    if (fov > 0.f) {
        camera.verticalFov = fov;
        camera.fovFromCinemachine = true;
    }

    camera.valid = true;
    {
        std::lock_guard<std::mutex> lock(freecamPoseMutex);
        if (freecamPose.valid) {
            const float currentFov = camera.verticalFov;
            camera = freecamPose;
            camera.verticalFov = currentFov;
        }
    }
    return camera;
}

bool BuildViewProjection(const CameraState& camera, float aspect,
    Matrix4x4& out, float nearPlane, float farPlane)
{
    if (!camera.valid || !std::isfinite(aspect) || aspect <= 0.01f || aspect > 100.f) return false;
    if (!std::isfinite(nearPlane) || !std::isfinite(farPlane) || !(nearPlane > 0.001f) || !(farPlane > nearPlane)) return false;
    if (!Finite(camera.position) || !Finite(camera.forward) || !Finite(camera.right) || !Finite(camera.up)) return false;
    if (!std::isfinite(camera.verticalFov) || camera.verticalFov < 1.0f || camera.verticalFov > 179.0f) return false;

    const Vector3& p = camera.position;
    const Vector3& r = camera.right;
    const Vector3& u = camera.up;
    const Vector3& f = camera.forward;

    // Row-vector convention: clip = [x y z 1] * view * projection, matching
    // WorldToScreen's column reads and CameraFinder's native-matrix fallback.
    // Unity stores matrices column-major, so this is the transposed view and
    // projection in memory order.
    const float dotRP = r.x * p.x + r.y * p.y + r.z * p.z;
    const float dotUP = u.x * p.x + u.y * p.y + u.z * p.z;
    const float dotFP = f.x * p.x + f.y * p.y + f.z * p.z;

    Matrix4x4 view{};
    view.matrix[0][0] = r.x;    view.matrix[0][1] = u.x;    view.matrix[0][2] = -f.x; view.matrix[0][3] = 0.f;
    view.matrix[1][0] = r.y;    view.matrix[1][1] = u.y;    view.matrix[1][2] = -f.y; view.matrix[1][3] = 0.f;
    view.matrix[2][0] = r.z;    view.matrix[2][1] = u.z;    view.matrix[2][2] = -f.z; view.matrix[2][3] = 0.f;
    view.matrix[3][0] = -dotRP; view.matrix[3][1] = -dotUP; view.matrix[3][2] = dotFP; view.matrix[3][3] = 1.f;

    const float fovRadians = camera.verticalFov * 3.14159265358979323846f / 180.f;
    const float halfTan = std::tan(fovRadians * 0.5f);
    if (!std::isfinite(halfTan) || halfTan <= 0.0001f) return false;

    const float yScale = 1.f / halfTan;
    const float xScale = yScale / aspect;
    const float a = (farPlane + nearPlane) / (nearPlane - farPlane);
    const float b = (2.f * farPlane * nearPlane) / (nearPlane - farPlane);

    Matrix4x4 projection{};
    projection.matrix[0][0] = xScale;
    projection.matrix[1][1] = yScale;
    projection.matrix[2][2] = a;
    projection.matrix[2][3] = -1.f;
    projection.matrix[3][2] = b;

    Matrix4x4 result = view * projection;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            if (!std::isfinite(result.matrix[row][col])) return false;
        }
    }

    out = result;
    return true;
}

bool WorldToScreen(const Matrix4x4& viewProjection, const Vector3& world,
    int screenWidth, int screenHeight, Vector2& out)
{
    if (screenWidth <= 0 || screenHeight <= 0) return false;
    if (!Finite(world)) return false;

    const float clipX = viewProjection.matrix[0][0] * world.x +
        viewProjection.matrix[1][0] * world.y +
        viewProjection.matrix[2][0] * world.z +
        viewProjection.matrix[3][0];
    const float clipY = viewProjection.matrix[0][1] * world.x +
        viewProjection.matrix[1][1] * world.y +
        viewProjection.matrix[2][1] * world.z +
        viewProjection.matrix[3][1];
    const float clipW = viewProjection.matrix[0][3] * world.x +
        viewProjection.matrix[1][3] * world.y +
        viewProjection.matrix[2][3] * world.z +
        viewProjection.matrix[3][3];

    // Gate: reject non-finite and points behind/at the near-clipping plane (< 0.05m)
    // where perspective divide creates massive coordinate explosions.
    if (!std::isfinite(clipX) || !std::isfinite(clipY) ||
        !std::isfinite(clipW) || clipW <= 0.05f) {
        return false;
    }

    const float ndcX = clipX / clipW;
    const float ndcY = clipY / clipW;
    if (!std::isfinite(ndcX) || !std::isfinite(ndcY)) return false;

    // Reject extreme NDC coordinates far outside the frustum to avoid numerical instability
    if (std::abs(ndcX) > 35.0f || std::abs(ndcY) > 35.0f) return false;

    const float sx = (screenWidth * 0.5f) * (1.f + ndcX);
    const float sy = (screenHeight * 0.5f) * (1.f - ndcY);
    if (!std::isfinite(sx) || !std::isfinite(sy)) return false;

    // Sanity boundary check: protect ImGui / DirectX from overflow
    constexpr float kMaxCoord = 50000.0f;
    if (sx < -kMaxCoord || sx > static_cast<float>(screenWidth) + kMaxCoord ||
        sy < -kMaxCoord || sy > static_cast<float>(screenHeight) + kMaxCoord) {
        return false;
    }

    out.x = sx;
    out.y = sy;
    return true;
}

} // namespace RobRunCamera
