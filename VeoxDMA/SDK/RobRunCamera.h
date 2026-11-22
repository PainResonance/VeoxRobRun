#pragma once
#include <cstdint>
#include "Matrix4x4.h"
#include "../Misc/Vector.h"

namespace RobRunCamera {

struct CameraState {
    bool valid = false;
    Vector3 position{};
    float orientation[4]{}; // x y z w
    float look[4]{};        // x y z w
    Vector3 forward{};
    Vector3 right{};
    Vector3 up{};
    float distance = 0.f;
    float verticalFov = 60.f;
    bool fovFromCinemachine = false;
    bool usedPhysical = false;
};

void Init(std::uintptr_t gameAssemblyBase);

// Owned by the aim worker, including restoration on shutdown.
void TickFreecam(bool& enabled, float& speed, std::uint64_t session, const CameraState& eye, std::uintptr_t views, std::uintptr_t game);
void StopFreecam(const char* reason = nullptr);
const char* GetFreecamStatus();

// Reads BattleCameraModel and attaches the active Cinemachine vertical FOV.
CameraState Read(std::uintptr_t battleCameraModel);

// Reads CinemachineCore -> brain list -> active CinemachineBrain +0xB0.
// Returns 0 when the path is unavailable.
float ReadCinemachineFov();

// Returns cached active CinemachineBrain pointer (resolving if needed).
std::uintptr_t GetCachedBrain(std::uintptr_t views = 0, std::uintptr_t game = 0);

// Builds a row-vector view-projection matrix matching RobRunReader's
// WorldToScreen convention. aspect = viewport width / viewport height.
bool BuildViewProjection(const CameraState& camera, float aspect,
    Matrix4x4& out, float nearPlane = 0.05f, float farPlane = 20000.f);

bool WorldToScreen(const Matrix4x4& viewProjection, const Vector3& world,
    int screenWidth, int screenHeight, Vector2& out);

} // namespace RobRunCamera
