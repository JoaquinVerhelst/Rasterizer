#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"

namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle):
			origin{_origin},
			fovAngle{_fovAngle}
		{
		}


		Vector3 origin{};
		float fovAngle{90.f};
		float fov{ tanf((fovAngle * TO_RADIANS) / 2.f) };

		Vector3 forward{Vector3::UnitZ};
		Vector3 up{Vector3::UnitY};
		Vector3 right{Vector3::UnitX};

		float totalPitch{};
		float totalYaw{};

		Matrix invViewMatrix{};
		Matrix viewMatrix{};

		Matrix projectionMatrix{};

		float movementSpeed{ 4.f };
		float rotationSpeed{ 10.f * TO_RADIANS };

		float nearPlane{ 0.1f };
		float farPlane{ 100.f };
		float aspectRatio{};

		void Initialize(float _fovAngle = 90.f, Vector3 _origin = {0.f,0.f,0.f}, float _aspecRatio = 1.f)
		{
			fovAngle = _fovAngle;
			fov = tanf((fovAngle * TO_RADIANS) / 2.f);

			origin = _origin;

			aspectRatio = _aspecRatio;
		}

		void CalculateViewMatrix()
		{
			//TODO W1
			//ONB => invViewMatrix
			//Inverse(ONB) => ViewMatrix

			const Matrix finalRotation = Matrix::CreateRotation({ totalPitch, totalYaw, 0.f });

			forward = finalRotation.TransformVector(Vector3::UnitZ);
			right = Vector3::Cross(Vector3::UnitY, forward).Normalized();
			up = Vector3::Cross(forward, right).Normalized();

			invViewMatrix = { right, up, forward, origin };

			viewMatrix = invViewMatrix.Inverse();


			//ViewMatrix => Matrix::CreateLookAtLH(...) [not implemented yet]
			//DirectX Implementation => https://learn.microsoft.com/en-us/windows/win32/direct3d9/d3dxmatrixlookatlh
		}

		void CalculateProjectionMatrix()
		{
			//TODO W2
			projectionMatrix = Matrix::CreatePerspectiveFovLH(fov, aspectRatio, nearPlane, farPlane);

	
			//DirectX Implementation => https://learn.microsoft.com/en-us/windows/win32/direct3d9/d3dxmatrixperspectivefovlh
		}

		void Update(Timer* pTimer)
		{


			//Camera Update Logic
			const float deltaTime = pTimer->GetElapsed();

			float currentMovementSpeed{ movementSpeed };

			//Keyboard Input

			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);

			if (pKeyboardState[SDL_SCANCODE_LSHIFT] == 1)
			{
				currentMovementSpeed *= 4;
			}


			if (pKeyboardState[SDL_SCANCODE_W] == 1)
				origin += currentMovementSpeed * forward * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_S] == 1)
				origin -= currentMovementSpeed * forward * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_A] == 1)
				origin -= right * currentMovementSpeed * deltaTime;
			if (pKeyboardState[SDL_SCANCODE_D] == 1)
				origin += right * currentMovementSpeed * deltaTime;



			//Mouse Input

			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);


			switch (mouseState)
			{
			case SDL_BUTTON_LMASK:

				origin -= forward * (mouseY * currentMovementSpeed / 2 * deltaTime);
				totalYaw += mouseX * rotationSpeed * deltaTime;
				break;
			case SDL_BUTTON_RMASK:

				totalYaw += mouseX * rotationSpeed * deltaTime;
				totalPitch -= mouseY * rotationSpeed * deltaTime;
				break;
			case SDL_BUTTON_X2:

				origin.y -= mouseY * currentMovementSpeed / 2 * deltaTime;
				break;

			}


			Matrix finalRotation = Matrix::CreateRotationX(totalPitch) * Matrix::CreateRotationY(totalYaw);

			forward = finalRotation.TransformVector(Vector3::UnitZ);
			forward.Normalize();



			//Update Matrices
			CalculateViewMatrix();
			CalculateProjectionMatrix(); //Try to optimize this - should only be called once or when fov/aspectRatio changes
		}
	};
}
