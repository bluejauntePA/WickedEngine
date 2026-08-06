#include "stdafx.h"
#include "Translator.h"
#include "wiRenderer.h"
#include "wiInput.h"
#include "wiMath.h"
#include "shaders/ShaderInterop_Renderer.h"
#include "wiEventHandler.h"

using namespace wi::ecs;
using namespace wi::scene;
using namespace wi::graphics;
using namespace wi::primitive;

namespace Translator_Internal
{
	PipelineState pso_solidpart;
	PipelineState pso_wirepart;
	constexpr float origin_size = 0.2f;
	constexpr float axis_length = 3.5f;
	constexpr float plane_min = 0.5f;
	constexpr float plane_max = 1.5f;
	constexpr float circle_radius = axis_length;
	constexpr float circle_width = 1.0f;
	constexpr float circle2_radius = circle_radius + 0.7f;
	constexpr float circle2_width = 0.3f;
	constexpr float pick_tolerance = 0.3f;

	XMVECTOR IndependentRotationAxis(const Translator& translator, Translator::TRANSLATOR_STATE state)
	{
		const XMFLOAT3* value = &translator.tool_rotation_axis_x;
		if (state == Translator::TRANSLATOR_Y) value = &translator.tool_rotation_axis_y;
		else if (state == Translator::TRANSLATOR_Z) value = &translator.tool_rotation_axis_z;
		XMVECTOR axis = XMLoadFloat3(value);
		return XMVectorGetX(XMVector3LengthSq(axis)) > 0.0000001f
			? XMVector3Normalize(axis) : XMVectorSet(1, 0, 0, 0);
	}

	XMMATRIX RotationAxisFrame(XMVECTOR axis)
	{
		axis = XMVector3Normalize(axis);
		XMVECTOR helper = std::abs(XMVectorGetX(XMVector3Dot(axis, XMVectorSet(0, 1, 0, 0)))) < 0.9f
			? XMVectorSet(0, 1, 0, 0) : XMVectorSet(0, 0, 1, 0);
		XMVECTOR zAxis = XMVector3Normalize(XMVector3Cross(axis, helper));
		XMVECTOR yAxis = XMVector3Normalize(XMVector3Cross(zAxis, axis));
		return XMMatrixSet(
			XMVectorGetX(axis), XMVectorGetY(axis), XMVectorGetZ(axis), 0,
			XMVectorGetX(yAxis), XMVectorGetY(yAxis), XMVectorGetZ(yAxis), 0,
			XMVectorGetX(zAxis), XMVectorGetY(zAxis), XMVectorGetZ(zAxis), 0,
			0, 0, 0, 1);
	}

	float PointSegmentDistanceSquared(
		const XMFLOAT2& point,
		const XMFLOAT2& start,
		const XMFLOAT2& end)
	{
		const float dx = end.x - start.x;
		const float dy = end.y - start.y;
		const float lengthSquared = dx * dx + dy * dy;
		if (lengthSquared <= 0.000001f)
		{
			const float px = point.x - start.x;
			const float py = point.y - start.y;
			return px * px + py * py;
		}
		const float t = std::clamp(
			((point.x - start.x) * dx + (point.y - start.y) * dy) / lengthSquared,
			0.0f, 1.0f);
		const float px = point.x - (start.x + dx * t);
		const float py = point.y - (start.y + dy * t);
		return px * px + py * py;
	}

	struct PlaneHandlePick
	{
		bool hit = false;
		float screenDistanceSquared = std::numeric_limits<float>::max();
		float depth = std::numeric_limits<float>::max();
	};

	PlaneHandlePick PickPlaneHandle(
		const CameraComponent& camera,
		const wi::Canvas& canvas,
		const XMFLOAT4& currentMouse,
		XMVECTOR corner0,
		XMVECTOR corner1,
		XMVECTOR corner2,
		XMVECTOR corner3)
	{
		PlaneHandlePick result;
		const float width = canvas.GetLogicalWidth();
		const float height = canvas.GetLogicalHeight();
		if (width <= 0 || height <= 0)
			return result;

		const XMMATRIX identity = XMMatrixIdentity();
		const XMMATRIX view = camera.GetView();
		const XMMATRIX projection = camera.GetProjection();
		XMFLOAT3 projected3[4];
		const XMVECTOR corners[4] = { corner0, corner1, corner2, corner3 };
		for (uint32_t i = 0; i < 4; ++i)
		{
			XMStoreFloat3(&projected3[i], XMVector3Project(
				corners[i], 0, 0, width, height, 0.0f, 1.0f,
				projection, view, identity));
			if (projected3[i].z < 0.0f || projected3[i].z > 1.0f)
				return result;
			result.depth = std::min(result.depth, projected3[i].z);
		}

		XMFLOAT2 projected[4];
		for (uint32_t i = 0; i < 4; ++i)
			projected[i] = XMFLOAT2(projected3[i].x, projected3[i].y);
		const XMFLOAT2 mouse(currentMouse.x, currentMouse.y);

		bool hasPositiveCross = false;
		bool hasNegativeCross = false;
		float doubleArea = 0.0f;
		for (uint32_t i = 0; i < 4; ++i)
		{
			const XMFLOAT2& start = projected[i];
			const XMFLOAT2& end = projected[(i + 1) % 4];
			const float cross = (end.x - start.x) * (mouse.y - start.y)
				- (end.y - start.y) * (mouse.x - start.x);
			hasPositiveCross |= cross > 0.01f;
			hasNegativeCross |= cross < -0.01f;
			doubleArea += start.x * end.y - end.x * start.y;
			result.screenDistanceSquared = std::min(
				result.screenDistanceSquared,
				PointSegmentDistanceSquared(mouse, start, end));
		}

		const bool hasArea = std::abs(doubleArea) > 1.0f;
		const bool inside = hasArea && !(hasPositiveCross && hasNegativeCross);
		if (inside)
			result.screenDistanceSquared = 0.0f;
		constexpr float edgeTolerancePixels = 5.0f;
		result.hit = inside || result.screenDistanceSquared
			<= edgeTolerancePixels * edgeTolerancePixels;
		return result;
	}

	struct RotationRingPick
	{
		bool hit = false;
		float screenDistanceSquared = std::numeric_limits<float>::max();
		float depth = std::numeric_limits<float>::max();
	};

	RotationRingPick PickRotationRing(
		const CameraComponent& camera,
		const wi::Canvas& canvas,
		const XMFLOAT4& currentMouse,
		XMVECTOR pivot,
		XMVECTOR normal,
		float scale,
		float ringWidth,
		bool frontFacingOnly)
	{
		RotationRingPick result;
		const float width = canvas.GetLogicalWidth();
		const float height = canvas.GetLogicalHeight();
		if (width <= 0 || height <= 0)
			return result;

		normal = XMVector3Normalize(normal);
		const XMMATRIX frame = RotationAxisFrame(normal);
		const XMMATRIX view = camera.GetView();
		const XMMATRIX projection = camera.GetProjection();
		const XMMATRIX identity = XMMatrixIdentity();
		const float ringRadius = circle_radius - ringWidth * 0.5f;
		const XMVECTOR viewToCamera = camera.GetEye() - pivot;
		const XMVECTOR planarView = viewToCamera
			- normal * XMVectorGetX(XMVector3Dot(viewToCamera, normal));
		const bool cullBack = frontFacingOnly
			&& XMVectorGetX(XMVector3LengthSq(planarView)) > 0.000001f;

		auto worldPoint = [&](float angle) {
			const XMVECTOR local = XMVectorSet(
				0, std::sin(angle) * ringRadius, std::cos(angle) * ringRadius, 0);
			return pivot + XMVector3TransformNormal(local, frame) * scale;
		};
		auto projectPoint = [&](XMVECTOR point, XMFLOAT3& projected) {
			XMStoreFloat3(&projected, XMVector3Project(
				point, 0, 0, width, height, 0.0f, 1.0f,
				projection, view, identity));
			return projected.z >= 0.0f && projected.z <= 1.0f;
		};

		XMFLOAT3 centerScreen;
		XMFLOAT3 radiusScreen0;
		XMFLOAT3 radiusScreen1;
		if (!projectPoint(pivot, centerScreen)
			|| !projectPoint(worldPoint(0), radiusScreen0)
			|| !projectPoint(worldPoint(XM_PIDIV2), radiusScreen1))
			return result;
		const auto screenRadius = [&](const XMFLOAT3& point) {
			const float dx = point.x - centerScreen.x;
			const float dy = point.y - centerScreen.y;
			return std::sqrt(dx * dx + dy * dy);
		};
		const float radiusPixels = std::max(
			screenRadius(radiusScreen0), screenRadius(radiusScreen1));
		const float rangePixels = std::max(4.0f,
			radiusPixels * std::max(
				ringWidth * 0.5f, pick_tolerance) / circle_radius);
		const XMFLOAT2 mouse(currentMouse.x, currentMouse.y);

		constexpr uint32_t segmentCount = 90;
		for (uint32_t i = 0; i < segmentCount; ++i)
		{
			const float angle0 = (float)i / (float)segmentCount * XM_2PI;
			const float angle1 = (float)(i + 1) / (float)segmentCount * XM_2PI;
			const float middle = (angle0 + angle1) * 0.5f;
			if (cullBack)
			{
				const XMVECTOR midpoint = worldPoint(middle) - pivot;
				if (XMVectorGetX(XMVector3Dot(midpoint, planarView)) < 0)
					continue;
			}

			XMFLOAT3 point0;
			XMFLOAT3 point1;
			if (!projectPoint(worldPoint(angle0), point0)
				|| !projectPoint(worldPoint(angle1), point1))
				continue;
			const float distanceSquared = PointSegmentDistanceSquared(
				mouse, XMFLOAT2(point0.x, point0.y), XMFLOAT2(point1.x, point1.y));
			if (distanceSquared < result.screenDistanceSquared)
			{
				result.screenDistanceSquared = distanceSquared;
				result.depth = (point0.z + point1.z) * 0.5f;
			}
		}

		result.hit = result.screenDistanceSquared <= rangePixels * rangePixels;
		return result;
	}

	void LoadShaders()
	{
		GraphicsDevice* device = wi::graphics::GetDevice();

		{
			PipelineStateDesc desc;

			desc.vs = wi::renderer::GetShader(wi::enums::VSTYPE_VERTEXCOLOR);
			desc.ps = wi::renderer::GetShader(wi::enums::PSTYPE_VERTEXCOLOR);
			desc.il = wi::renderer::GetInputLayout(wi::enums::ILTYPE_VERTEXCOLOR);
			desc.dss = wi::renderer::GetDepthStencilState(wi::enums::DSSTYPE_DEFAULT);
			desc.rs = wi::renderer::GetRasterizerState(wi::enums::RSTYPE_DOUBLESIDED);
			desc.bs = wi::renderer::GetBlendState(wi::enums::BSTYPE_TRANSPARENT);
			desc.pt = PrimitiveTopology::TRIANGLELIST;

			device->CreatePipelineState(&desc, &pso_solidpart);
		}

		{
			PipelineStateDesc desc;

			desc.vs = wi::renderer::GetShader(wi::enums::VSTYPE_VERTEXCOLOR);
			desc.ps = wi::renderer::GetShader(wi::enums::PSTYPE_VERTEXCOLOR);
			desc.il = wi::renderer::GetInputLayout(wi::enums::ILTYPE_VERTEXCOLOR);
			desc.dss = wi::renderer::GetDepthStencilState(wi::enums::DSSTYPE_DEFAULT);
			desc.rs = wi::renderer::GetRasterizerState(wi::enums::RSTYPE_WIRE_DOUBLESIDED);
			desc.bs = wi::renderer::GetBlendState(wi::enums::BSTYPE_TRANSPARENT);
			desc.pt = PrimitiveTopology::LINELIST;

			device->CreatePipelineState(&desc, &pso_wirepart);
		}
	}


	struct Vertex
	{
		XMFLOAT4 position;
		XMFLOAT4 color;
	};
	const Vertex cubeVerts[] = {
		{XMFLOAT4(-1,1,1,1),   XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,-1,1,1),  XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,-1,-1,1), XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,1,1,1),	XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,-1,1,1),   XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,-1,1,1),  XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,1,-1,1),   XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,-1,-1,1),  XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,-1,1,1),   XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,1,-1,1),  XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,-1,-1,1), XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,-1,-1,1),  XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,-1,1,1),  XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,-1,1,1),   XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,-1,-1,1),  XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,1,1,1),	XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,1,1,1),   XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,1,-1,1),  XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,1,-1,1),  XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,1,1,1),   XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,-1,-1,1), XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,1,1,1),   XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,1,1,1),	XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,-1,1,1),  XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,1,1,1),	XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,1,-1,1),   XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,-1,1,1),   XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,1,-1,1),   XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,1,-1,1),  XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,-1,-1,1),  XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,-1,-1,1), XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,-1,1,1),  XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,-1,-1,1),  XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,1,-1,1),   XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(1,1,1,1),	XMFLOAT4(1,1,1,1)},
		{XMFLOAT4(-1,1,-1,1),  XMFLOAT4(1,1,1,1)},
	};
}
using namespace Translator_Internal;

void Translator::Update(const CameraComponent& camera, const XMFLOAT4& currentMouse, const wi::Canvas& canvas)
{
	if (selected.empty())
	{
		transform.ClearTransform();
		transform.UpdateTransform();
		state = TRANSLATOR_IDLE;
		return;
	}

	dragStarted = false;
	dragEnded = false;

	XMVECTOR pos = transform.GetPositionV();

	// Non recursive selection will be computed to not apply recursive operations two times
	//	A recursive operation is for example translating a parent transform
	//	An other recursive operation is serializing selected parent entities
	Scene& scene = *this->scene;
	selectedEntitiesLookup.clear();
	for (auto& x : selected)
	{
		selectedEntitiesLookup.insert(x.entity);
	}
	selectedEntitiesNonRecursive.clear();
	for (auto& x : selected)
	{
		const HierarchyComponent* hier = scene.hierarchy.GetComponent(x.entity);
		if (hier == nullptr || selectedEntitiesLookup.count(hier->parentID) == 0)
		{
			selectedEntitiesNonRecursive.push_back(x.entity);
		}
	}

	if (IsEnabled())
	{
		PreTranslate();
		if (!has_selected_transform)
		{
			state = TRANSLATOR_IDLE;
			return;
		}

		if (!dragging)
		{
			const XMFLOAT3 p = transform.GetPosition();
			// Size from the camera projection, not from the manipulator interaction
			// mode. A perspective viewport can use the 2D screen-space rotation ring,
			// and it still needs distance scaling to remain constant on screen.
			dist = camera.IsOrtho()
				? camera.ortho_vertical_size * 0.025f * tool_scale
				: std::max(wi::math::Distance(p, camera.Eye) * 0.05f, 0.0001f) * tool_scale;
		}

		if (!tool_interaction_enabled)
		{
			state = TRANSLATOR_IDLE;
			dragging = false;
			return;
		}

		const Ray ray = wi::renderer::GetPickRay((long)currentMouse.x, (long)currentMouse.y, canvas, camera);
		const XMVECTOR rayOrigin = XMLoadFloat3(&ray.origin);
		const XMVECTOR rayDir = XMLoadFloat3(&ray.direction);

		if (!dragging)
		{
			state = TRANSLATOR_IDLE;

			// Decide which state to enter for dragging:
			XMFLOAT3 p = transform.GetPosition();

			if (isRotator)
			{
				XMMATRIX localRotation = GetLocalRotation();
				XMVECTOR localX = tool_use_independent_rotation_axes
					? IndependentRotationAxis(*this, TRANSLATOR_X)
					: XMVector3TransformNormal(XMVectorSet(1, 0, 0, 0), localRotation);
				XMVECTOR localY = tool_use_independent_rotation_axes
					? IndependentRotationAxis(*this, TRANSLATOR_Y)
					: XMVector3TransformNormal(XMVectorSet(0, 1, 0, 0), localRotation);
				XMVECTOR localZ = tool_use_independent_rotation_axes
					? IndependentRotationAxis(*this, TRANSLATOR_Z)
					: XMVector3TransformNormal(XMVectorSet(0, 0, 1, 0), localRotation);

				XMVECTOR plane_zy = XMPlaneFromPointNormal(pos, localX);
				XMVECTOR plane_xz = XMPlaneFromPointNormal(pos, localY);
				XMVECTOR plane_xy = XMPlaneFromPointNormal(pos, localZ);

				XMVECTOR intersection = XMPlaneIntersectLine(plane_zy, rayOrigin, rayOrigin + rayDir * camera.zFarP);
				float dist_x = XMVectorGetX(XMVector3LengthSq(intersection - rayOrigin));
				float len_x = XMVectorGetX(XMVector3Length(intersection - pos)) / dist;
				intersection = XMPlaneIntersectLine(plane_xz, rayOrigin, rayOrigin + rayDir * camera.zFarP);
				float dist_y = XMVectorGetX(XMVector3LengthSq(intersection - rayOrigin));
				float len_y = XMVectorGetX(XMVector3Length(intersection - pos)) / dist;
				intersection = XMPlaneIntersectLine(plane_xy, rayOrigin, rayOrigin + rayDir * camera.zFarP);
				float dist_z = XMVectorGetX(XMVector3LengthSq(intersection - rayOrigin));
				float len_z = XMVectorGetX(XMVector3Length(intersection - pos)) / dist;

				const float thick = tool_thickness;
				const float axisRingWidth = (tool_rotation_ring_tube_sides >= 3
					? circle2_width : circle_width) * thick;
				float range = std::max(axisRingWidth * 0.5f, pick_tolerance);
				float perimeter = circle_radius - axisRingWidth * 0.5f;
				float best_dist = std::numeric_limits<float>::max();

				if (!is2D)
				{
					if (tool_axis_x_enabled && std::abs(perimeter - len_x) <= range && dist_x < best_dist)
					{
						state = TRANSLATOR_X;
						XMStoreFloat3(&axis, localX);
						best_dist = dist_x;
					}
					if (tool_axis_y_enabled && std::abs(perimeter - len_y) <= range && dist_y < best_dist)
					{
						state = TRANSLATOR_Y;
						XMStoreFloat3(&axis, localY);
						best_dist = dist_y;
					}
				}
				if (tool_axis_z_enabled && std::abs(perimeter - len_z) <= range && dist_z < best_dist)
				{
					state = TRANSLATOR_Z;
					XMStoreFloat3(&axis, localZ);
					best_dist = dist_z;
				}

				if (!is2D && (tool_rotation_front_facing_only
					|| tool_rotation_ring_tube_sides >= 3))
				{
					state = TRANSLATOR_IDLE;
					float bestScreenDistance = std::numeric_limits<float>::max();
					float bestScreenDepth = std::numeric_limits<float>::max();
					auto tryRing = [&](TRANSLATOR_STATE candidate, XMVECTOR candidateAxis) {
						const RotationRingPick pick = PickRotationRing(
							camera, canvas, currentMouse, pos, candidateAxis, dist,
							axisRingWidth,
							tool_rotation_front_facing_only);
						if (!pick.hit)
							return;
						const bool closerToPointer = pick.screenDistanceSquared
							< bestScreenDistance - 1.0f;
						const bool samePointerDistance = std::abs(
							pick.screenDistanceSquared - bestScreenDistance) <= 1.0f;
						if (!closerToPointer
							&& !(samePointerDistance && pick.depth < bestScreenDepth))
							return;
						state = candidate;
						XMStoreFloat3(&axis, candidateAxis);
						bestScreenDistance = pick.screenDistanceSquared;
						bestScreenDepth = pick.depth;
					};
					if (tool_axis_x_enabled) tryRing(TRANSLATOR_X, localX);
					if (tool_axis_y_enabled) tryRing(TRANSLATOR_Y, localY);
					if (tool_axis_z_enabled) tryRing(TRANSLATOR_Z, localZ);
				}

				XMVECTOR screen_normal = XMVector3Normalize(camera.GetEye() - pos);
				XMVECTOR plane_screen = XMPlaneFromPointNormal(pos, screen_normal);
				intersection = XMPlaneIntersectLine(plane_screen, rayOrigin, rayOrigin + rayDir * camera.zFarP);
				float len_screen = XMVectorGetX(XMVector3Length(intersection - pos)) / dist;
				const float freeRingWidth = circle2_width * thick;
				range = std::max(freeRingWidth * 0.5f, pick_tolerance);
				perimeter = circle2_radius - freeRingWidth * 0.5f;
				if (tool_trackball_enabled
					&& (tool_axis_x_enabled || tool_axis_y_enabled || tool_axis_z_enabled)
					&& len_screen <= tool_trackball_radius)
				{
					state = TRANSLATOR_TRACKBALL;
				}
				else if ((tool_axis_x_enabled || tool_axis_y_enabled || tool_axis_z_enabled)
					&& std::abs(perimeter - len_screen) <= range)
				{
					state = TRANSLATOR_XYZ;
					XMStoreFloat3(&axis, screen_normal);
				}

			}
			else
			{
				XMMATRIX localRotation = GetLocalRotation();
				AABB aabb_origin;
				aabb_origin.createFromHalfWidth(p, XMFLOAT3(origin_size * dist, origin_size * dist, origin_size * dist));

				XMFLOAT3 maxp;
				XMStoreFloat3(&maxp, pos + XMVector3Transform(XMVectorSet(axis_length, 0, 0, 0) * dist, localRotation * GetMirrorMatrix(TRANSLATOR_X, camera)));
				AABB aabb_x = AABB::Merge(AABB(wi::math::Min(p, maxp), wi::math::Max(p, maxp)), aabb_origin);

				XMStoreFloat3(&maxp, pos + XMVector3Transform(XMVectorSet(0, axis_length, 0, 0) * dist, localRotation * GetMirrorMatrix(TRANSLATOR_Y, camera)));
				AABB aabb_y = AABB::Merge(AABB(wi::math::Min(p, maxp), wi::math::Max(p, maxp)), aabb_origin);

				XMStoreFloat3(&maxp, pos + XMVector3Transform(XMVectorSet(0, 0, axis_length, 0) * dist, localRotation * GetMirrorMatrix(TRANSLATOR_Z, camera)));
				AABB aabb_z = AABB::Merge(AABB(wi::math::Min(p, maxp), wi::math::Max(p, maxp)), aabb_origin);

				XMFLOAT3 minp;
				XMStoreFloat3(&minp, pos + XMVector3Transform(XMVectorSet(plane_min, plane_min, 0, 0) * dist, localRotation * GetMirrorMatrix(TRANSLATOR_XY, camera)));
				XMStoreFloat3(&maxp, pos + XMVector3Transform(XMVectorSet(plane_max, plane_max, 0, 0) * dist, localRotation * GetMirrorMatrix(TRANSLATOR_XY, camera)));
				AABB aabb_xy = AABB(wi::math::Min(minp, maxp), wi::math::Max(minp, maxp));

				XMStoreFloat3(&minp, pos + XMVector3Transform(XMVectorSet(plane_min, 0, plane_min, 0) * dist, localRotation * GetMirrorMatrix(TRANSLATOR_XZ, camera)));
				XMStoreFloat3(&maxp, pos + XMVector3Transform(XMVectorSet(plane_max, 0, plane_max, 0) * dist, localRotation * GetMirrorMatrix(TRANSLATOR_XZ, camera)));
				AABB aabb_xz = AABB(wi::math::Min(minp, maxp), wi::math::Max(minp, maxp));

				XMStoreFloat3(&minp, pos + XMVector3Transform(XMVectorSet(0, plane_min, plane_min, 0) * dist, localRotation * GetMirrorMatrix(TRANSLATOR_YZ, camera)));
				XMStoreFloat3(&maxp, pos + XMVector3Transform(XMVectorSet(0, plane_max, plane_max, 0) * dist, localRotation * GetMirrorMatrix(TRANSLATOR_YZ, camera)));
				AABB aabb_yz = AABB(wi::math::Min(minp, maxp), wi::math::Max(minp, maxp));

				if (aabb_origin.intersects(ray))
				{
					state = TRANSLATOR_XYZ;
				}
				else if (tool_axis_x_enabled && aabb_x.intersects(ray))
				{
					state = TRANSLATOR_X;
				}
				else if (tool_axis_y_enabled && aabb_y.intersects(ray))
				{
					state = TRANSLATOR_Y;
				}
				else if (tool_axis_z_enabled && !is2D && aabb_z.intersects(ray))
				{
					state = TRANSLATOR_Z;
				}

				if (isTranslator && tool_use_screen_space_plane_picking)
				{
					float bestScreenDistance = std::numeric_limits<float>::max();
					float bestScreenDepth = std::numeric_limits<float>::max();
					auto tryPlane = [&](TRANSLATOR_STATE candidate,
						XMVECTOR corner0, XMVECTOR corner1,
						XMVECTOR corner2, XMVECTOR corner3) {
						const PlaneHandlePick pick = PickPlaneHandle(
							camera, canvas, currentMouse,
							corner0, corner1, corner2, corner3);
						if (!pick.hit)
							return;
						const bool closerToPointer = pick.screenDistanceSquared
							< bestScreenDistance - 1.0f;
						const bool samePointerDistance = std::abs(
							pick.screenDistanceSquared - bestScreenDistance) <= 1.0f;
						if (!closerToPointer
							&& !(samePointerDistance && pick.depth < bestScreenDepth))
							return;
						state = candidate;
						bestScreenDistance = pick.screenDistanceSquared;
						bestScreenDepth = pick.depth;
					};
					auto planeCorner = [&](TRANSLATOR_STATE candidate,
						float x, float y, float z) {
						return pos + XMVector3Transform(
							XMVectorSet(x, y, z, 0) * dist,
							localRotation * GetMirrorMatrix(candidate, camera));
					};

					if (tool_axis_x_enabled && tool_axis_y_enabled)
					{
						tryPlane(TRANSLATOR_XY,
							planeCorner(TRANSLATOR_XY, plane_min, plane_min, 0),
							planeCorner(TRANSLATOR_XY, plane_max, plane_min, 0),
							planeCorner(TRANSLATOR_XY, plane_max, plane_max, 0),
							planeCorner(TRANSLATOR_XY, plane_min, plane_max, 0));
					}
					if (!is2D && tool_axis_x_enabled && tool_axis_z_enabled)
					{
						tryPlane(TRANSLATOR_XZ,
							planeCorner(TRANSLATOR_XZ, plane_min, 0, plane_min),
							planeCorner(TRANSLATOR_XZ, plane_max, 0, plane_min),
							planeCorner(TRANSLATOR_XZ, plane_max, 0, plane_max),
							planeCorner(TRANSLATOR_XZ, plane_min, 0, plane_max));
					}
					if (!is2D && tool_axis_y_enabled && tool_axis_z_enabled)
					{
						tryPlane(TRANSLATOR_YZ,
							planeCorner(TRANSLATOR_YZ, 0, plane_min, plane_min),
							planeCorner(TRANSLATOR_YZ, 0, plane_max, plane_min),
							planeCorner(TRANSLATOR_YZ, 0, plane_max, plane_max),
							planeCorner(TRANSLATOR_YZ, 0, plane_min, plane_max));
					}
				}
				else if (isTranslator && state != TRANSLATOR_XYZ)
				{
					// these can overlap, so take closest one (by checking plane ray trace distance):
					XMVECTOR N = XMVector3TransformNormal(XMVectorSet(0, 0, 1, 0), localRotation);

					float prio = FLT_MAX;
					if (tool_axis_x_enabled && tool_axis_y_enabled && aabb_xy.intersects(ray))
					{
						state = TRANSLATOR_XY;
						prio = XMVectorGetX(XMVector3Dot(N, (rayOrigin - pos) / XMVectorAbs(XMVector3Dot(N, rayDir))));
					}

					N = XMVector3TransformNormal(XMVectorSet(0, 1, 0, 0), localRotation);
					float d = XMVectorGetX(XMVector3Dot(N, (rayOrigin - pos) / XMVectorAbs(XMVector3Dot(N, rayDir))));
					if (tool_axis_x_enabled && tool_axis_z_enabled && d < prio && aabb_xz.intersects(ray))
					{
						state = TRANSLATOR_XZ;
						prio = d;
					}

					N = XMVector3TransformNormal(XMVectorSet(1, 0, 0, 0), localRotation);
					d = XMVectorGetX(XMVector3Dot(N, (rayOrigin - pos) / XMVectorAbs(XMVector3Dot(N, rayDir))));
					if (tool_axis_y_enabled && tool_axis_z_enabled && d < prio && aabb_yz.intersects(ray))
					{
						state = TRANSLATOR_YZ;
					}
				}
			}
		}

		if (dragging || (state != TRANSLATOR_IDLE && (wi::input::Press(wi::input::MOUSE_BUTTON_LEFT) || wi::input::IsTouchPanning())))
		{
			// Dragging operation:
			if (isRotator)
			{
				if (state == TRANSLATOR_TRACKBALL)
				{
					if (!dragging)
					{
						dragStarted = true;
						transform_start = transform;
						trackball_mouse_start = XMFLOAT2(currentMouse.x, currentMouse.y);
						matrices_start = matrices_current;
					}

					trackball_delta_radians = XMFLOAT2(
						(trackball_mouse_start.y - currentMouse.y) * tool_trackball_sensitivity,
						(trackball_mouse_start.x - currentMouse.x) * tool_trackball_sensitivity);
					if (wi::input::Down(wi::input::BUTTON::KEYBOARD_BUTTON_LCONTROL))
					{
						constexpr float trackballSnap = XM_PI / 36.0f;
						trackball_delta_radians.x = std::round(
							trackball_delta_radians.x / trackballSnap) * trackballSnap;
						trackball_delta_radians.y = std::round(
							trackball_delta_radians.y / trackballSnap) * trackballSnap;
					}
					XMVECTOR trackballAxis = camera.GetRight() * trackball_delta_radians.x
						+ camera.GetUp() * trackball_delta_radians.y;
					trackball_angle = XMVectorGetX(XMVector3Length(trackballAxis));
					if (trackball_angle > 0.000001f)
					{
						trackballAxis = XMVector3Normalize(trackballAxis);
						XMStoreFloat3(&trackball_axis, trackballAxis);
					}
					transform = transform_start;
					if (trackball_angle > 0.000001f)
					{
						transform.Rotate(XMQuaternionRotationAxis(
							XMLoadFloat3(&trackball_axis), trackball_angle));
					}
				}
				else
				{
				XMVECTOR intersection = XMPlaneIntersectLine(XMPlaneFromPointNormal(pos, XMLoadFloat3(&axis)), rayOrigin, rayOrigin + rayDir * camera.zFarP);

				if (!dragging)
				{
					dragStarted = true;
					transform_start = transform;
					XMStoreFloat3(&intersection_start, intersection);
					matrices_start = matrices_current;
				}
				XMVECTOR intersectionPrev = XMLoadFloat3(&intersection_start);

				XMVECTOR o = XMVector3Normalize(intersectionPrev - pos);
				XMVECTOR c = XMVector3Normalize(intersection - pos);
				XMFLOAT3 original, current;
				XMStoreFloat3(&original, o);
				XMStoreFloat3(&current, c);
				angle = wi::math::GetAngle(original, current, axis);

				if (tool_use_independent_rotation_axes && state != TRANSLATOR_XYZ)
				{
					const XMMATRIX frame = RotationAxisFrame(XMLoadFloat3(&axis));
					XMFLOAT3 ref;
					XMStoreFloat3(&ref, XMVector3TransformNormal(XMVectorSet(0, 1, 0, 0), frame));
					angle_start = wi::math::GetAngle(ref, original, axis);
				}
				else switch (state)
				{
				case Translator::TRANSLATOR_X:
					angle_start = wi::math::GetAngle(XMFLOAT3(0, 1, 0), original, axis);
					break;
				case Translator::TRANSLATOR_Y:
					angle_start = wi::math::GetAngle(XMFLOAT3(0, 0, 1), original, axis);
					break;
				case Translator::TRANSLATOR_Z:
					angle_start = wi::math::GetAngle(XMFLOAT3(1, 0, 0), original, axis);
					break;
				case Translator::TRANSLATOR_XYZ:
					{
						XMMATRIX M = XMMatrixInverse(nullptr, XMMatrixLookToLH(XMVectorZero(), XMVector3Normalize(transform.GetPositionV() - camera.GetEye()), camera.GetUp()));
						XMFLOAT3 ref;
						XMStoreFloat3(&ref, XMVector3TransformNormal(XMVectorSet(0, 1, 0, 0), M));
						angle_start = wi::math::GetAngle(ref, original, axis);
					}
					break;
				default:
					break;
				}

				if (wi::input::Down(wi::input::BUTTON::KEYBOARD_BUTTON_LCONTROL))
				{
					// Snap mode:
					angle = std::round(angle / rotate_snap) * rotate_snap;
				}
				angle = std::fmod(angle, XM_2PI);

				transform = transform_start;
				transform.Rotate(XMQuaternionRotationAxis(XMLoadFloat3(&axis), angle));
				}
			}
			else
			{
				XMVECTOR plane, planeNormal;
				XMMATRIX localRotation = GetLocalRotation();
				if (state == TRANSLATOR_X)
				{
					XMVECTOR axis = XMVector3TransformNormal(XMVectorSet(1, 0, 0, 0), localRotation);
					XMVECTOR wrong = XMVector3Cross(camera.GetAt(), axis);
					planeNormal = XMVector3Cross(wrong, axis);
					XMStoreFloat3(&this->axis, axis);
				}
				else if (state == TRANSLATOR_Y)
				{
					XMVECTOR axis = XMVector3TransformNormal(XMVectorSet(0, 1, 0, 0), localRotation);
					XMVECTOR wrong = XMVector3Cross(camera.GetAt(), axis);
					planeNormal = XMVector3Cross(wrong, axis);
					XMStoreFloat3(&this->axis, axis);
				}
				else if (state == TRANSLATOR_Z)
				{
					XMVECTOR axis = XMVector3TransformNormal(XMVectorSet(0, 0, 1, 0), localRotation);
					XMVECTOR wrong = XMVector3Cross(camera.GetAt(), axis);
					planeNormal = XMVector3Cross(wrong, axis);
					XMStoreFloat3(&this->axis, axis);
				}
				else if (state == TRANSLATOR_XY)
				{
					planeNormal = XMVector3TransformNormal(XMVectorSet(0, 0, 1, 0), localRotation);
				}
				else if (state == TRANSLATOR_XZ)
				{
					planeNormal = XMVector3TransformNormal(XMVectorSet(0, 1, 0, 0), localRotation);
				}
				else if (state == TRANSLATOR_YZ)
				{
					planeNormal = XMVector3TransformNormal(XMVectorSet(1, 0, 0, 0), localRotation);
				}
				else
				{
					// xyz
					planeNormal = camera.GetAt();
				}

				// Use stored plane from drag start to avoid drift
				if (dragging)
				{
					plane = XMLoadFloat4(&drag_plane);
				}
				else
				{
					plane = XMPlaneFromPointNormal(pos, XMVector3Normalize(planeNormal));
				}

				// Check if ray is nearly parallel to plane - only abort if not already dragging
				if (XMVectorGetX(XMVectorAbs(XMVector3Dot(XMPlaneNormalize(plane), rayDir))) < 0.001f)
				{
					if (!dragging)
					{
						state = TRANSLATOR_IDLE;
						return;
					}
					// If already dragging, skip this frame's update but maintain state
					return;
				}
				XMVECTOR intersection = XMPlaneIntersectLine(plane, rayOrigin, rayOrigin + rayDir * camera.zFarP);

				if (!dragging)
				{
					dragStarted = true;
					transform_start = transform;
					XMStoreFloat3(&intersection_start, intersection);
					XMStoreFloat4(&drag_plane, plane);
					matrices_start = matrices_current;
				}
				XMVECTOR intersectionPrev = XMLoadFloat3(&intersection_start);

				XMVECTOR startPosition = transform_start.GetPositionV();
				XMVECTOR deltaV;
				if (state == TRANSLATOR_X)
				{
					XMVECTOR axisDir = XMVector3TransformNormal(XMVectorSet(1, 0, 0, 0), localRotation);
					XMVECTOR A = startPosition, B = startPosition + axisDir;
					XMVECTOR P = wi::math::ClosestPointOnLine(A, B, intersection);
					XMVECTOR PPrev = wi::math::ClosestPointOnLine(A, B, intersectionPrev);
					deltaV = P - PPrev;
				}
				else if (state == TRANSLATOR_Y)
				{
					XMVECTOR axisDir = XMVector3TransformNormal(XMVectorSet(0, 1, 0, 0), localRotation);
					XMVECTOR A = startPosition, B = startPosition + axisDir;
					XMVECTOR P = wi::math::ClosestPointOnLine(A, B, intersection);
					XMVECTOR PPrev = wi::math::ClosestPointOnLine(A, B, intersectionPrev);
					deltaV = P - PPrev;
				}
				else if (state == TRANSLATOR_Z)
				{
					XMVECTOR axisDir = XMVector3TransformNormal(XMVectorSet(0, 0, 1, 0), localRotation);
					XMVECTOR A = startPosition, B = startPosition + axisDir;
					XMVECTOR P = wi::math::ClosestPointOnLine(A, B, intersection);
					XMVECTOR PPrev = wi::math::ClosestPointOnLine(A, B, intersectionPrev);
					deltaV = P - PPrev;
				}
				else
				{
					deltaV = intersection - intersectionPrev;

					if (isScalator)
					{
						deltaV = XMVectorReplicate(XMVectorGetY(deltaV));
					}
				}

				transform = transform_start;
				if (isTranslator)
				{
					transform.Translate(deltaV);
				}
				if (isScalator)
				{
					deltaV = XMVector3TransformNormal(deltaV, GetMirrorMatrix(state, camera));
					deltaV = XMVector3Rotate(deltaV, transform.GetRotationV());
					XMFLOAT3 delta;
					XMStoreFloat3(&delta, deltaV);
					XMFLOAT3 scale = transform.GetScale();
					scale = wi::math::Max(scale, XMFLOAT3(0.001f, 0.001f, 0.001f)); // no zero division
					scale = XMFLOAT3((1.0f / scale.x) * (scale.x + delta.x), (1.0f / scale.y) * (scale.y + delta.y), (1.0f / scale.z) * (scale.z + delta.z));
					transform.Scale(scale);
				}

				if (wi::input::Down(wi::input::BUTTON::KEYBOARD_BUTTON_LCONTROL) || wi::input::Down(wi::input::BUTTON::KEYBOARD_BUTTON_RCONTROL))
				{
					// Snap to grid mode:
					if (isTranslator)
					{
						transform.translation_local.x = std::round(transform.translation_local.x / translate_snap) * translate_snap;
						transform.translation_local.y = std::round(transform.translation_local.y / translate_snap) * translate_snap;
						transform.translation_local.z = std::round(transform.translation_local.z / translate_snap) * translate_snap;
					}
					if (isScalator)
					{
						transform.scale_local.x = std::max(scale_snap, std::round(transform.scale_local.x / scale_snap) * scale_snap);
						transform.scale_local.y = std::max(scale_snap, std::round(transform.scale_local.y / scale_snap) * scale_snap);
						transform.scale_local.z = std::max(scale_snap, std::round(transform.scale_local.z / scale_snap) * scale_snap);
					}
				}
				if (wi::input::Down(wi::input::BUTTON::KEYBOARD_BUTTON_LSHIFT) || wi::input::Down(wi::input::BUTTON::KEYBOARD_BUTTON_RSHIFT))
				{
					// Snap to surface mode:

					// 1.) Collect all objects which could be in the hierarchy of any of the selected ones
					//	This is important because we could select a top parent that doesn't have object
					//	but I still want to disable children's filters
					selectedWithHierarchy.clear();
					for (size_t i = 0; i < scene.objects.GetCount(); ++i)
					{
						Entity entity = scene.objects.GetEntity(i);
						for (auto& potential_parent : selectedEntitiesNonRecursive)
						{
							if (entity == potential_parent || scene.Entity_IsDescendant(entity, potential_parent))
							{
								selectedWithHierarchy.push_back(entity);
								break;
							}
						}
					}

					// 2.) Disable all object filters which are part of selection hierarchy for the next picking
					//	This is to filter them out from picking, we only want to pick what's underneath
					temp_filters.reserve(selectedWithHierarchy.size());
					for (auto& entity : selectedWithHierarchy)
					{
						ObjectComponent* object = scene.objects.GetComponent(entity);
						if (object == nullptr)
							continue;
						temp_filters.push_back(uint64_t(object->filterMask) | (uint64_t(object->filterMaskDynamic) << 32ull));
						object->filterMask = 0;
						object->filterMaskDynamic = 0;
					}

					// 3.) Pick into scene to determine placement position
					Ray ray = wi::renderer::GetPickRay((long)currentMouse.x, (long)currentMouse.y, canvas, camera);
					wi::scene::Scene::RayIntersectionResult result = scene.Intersects(ray, wi::enums::FILTER_OBJECT_ALL);
					transform.translation_local = result.position;

					// 4.) Restore all filters to original values
					size_t ind = 0;
					for (auto& entity : selectedWithHierarchy)
					{
						ObjectComponent* object = scene.objects.GetComponent(entity);
						if (object == nullptr)
							continue;
						uint64_t tmp = temp_filters[ind++];
						object->filterMask = uint32_t(tmp);
						object->filterMaskDynamic = uint32_t(tmp >> 32ull);
					}
				}
			}

			transform.UpdateTransform();

			dragging = true;
		}

		if (isScalator || isRotator || isTranslator)
		{
			if (dragging)
			{
				PostTranslate();
			}
		}

		if (!wi::input::Down(wi::input::MOUSE_BUTTON_LEFT) && !wi::input::IsTouchPanning())
		{
			if (dragging)
			{
				dragEnded = true;
			}
			dragging = false;
		}
	}
	else
	{
		if (dragging)
		{
			dragEnded = true;
		}
		dragging = false;
		state = TRANSLATOR_IDLE;
	}
}
void Translator::Draw(const CameraComponent& camera, const XMFLOAT4& currentMouse, CommandList cmd) const
{
	if (!IsEnabled() || selected.empty() || !has_selected_transform)
	{
		return;
	}

	static bool shaders_loaded = false;
	if (!shaders_loaded)
	{
		shaders_loaded = true;
		static wi::eventhandler::Handle handle = wi::eventhandler::Subscribe(wi::eventhandler::EVENT_RELOAD_SHADERS, [](uint64_t userdata) { Translator_Internal::LoadShaders(); });
		Translator_Internal::LoadShaders();
	}

	GraphicsDevice* device = wi::graphics::GetDevice();

	device->EventBegin("Translator", cmd);

	CameraComponent cam_tmp = camera;
	cam_tmp.jitter = XMFLOAT2(0, 0); // remove temporal jitter
	cam_tmp.UpdateCamera();
	XMMATRIX VP = cam_tmp.GetViewProjection();

	MiscCB sb;

	const bool independentRotationAxes = isRotator && tool_use_independent_rotation_axes;
	XMMATRIX localRotation = independentRotationAxes ? XMMatrixIdentity() : GetLocalRotation();
	XMMATRIX mat = XMMatrixScaling(dist, dist, dist) * localRotation * XMMatrixTranslationFromVector(transform.GetPositionV()) * VP;
	XMMATRIX matX = independentRotationAxes
		? RotationAxisFrame(IndependentRotationAxis(*this, TRANSLATOR_X))
		: XMMatrixIdentity();
	XMMATRIX matY = independentRotationAxes
		? RotationAxisFrame(IndependentRotationAxis(*this, TRANSLATOR_Y))
		: XMMatrixRotationZ(XM_PIDIV2)*XMMatrixRotationY(XM_PIDIV2);
	XMMATRIX matZ = independentRotationAxes
		? RotationAxisFrame(IndependentRotationAxis(*this, TRANSLATOR_Z))
		: XMMatrixRotationY(-XM_PIDIV2)*XMMatrixRotationZ(-XM_PIDIV2);

	constexpr float channel_min = 0.25f; // min color channel, to avoid pure red/green/blue
	constexpr XMFLOAT4 highlight_color = XMFLOAT4(1, 0.6f, 0, 1);

	// Axes:
	if (isRotator && !is2D && (tool_rotation_front_facing_only
		|| tool_rotation_ring_tube_sides >= 3))
	{
		device->BindPipelineState(&pso_solidpart, cmd);
		const XMVECTOR pivot = transform.GetPositionV();
		const XMVECTOR viewToCamera = camera.GetEye() - pivot;
		const bool isolateActiveRing = dragging
			&& state != TRANSLATOR_IDLE && state != TRANSLATOR_XYZ;

		auto drawRing = [&](TRANSLATOR_STATE candidate, const XMMATRIX& axisFrame,
			const XMFLOAT4& baseColor) {
			if (dragging && state == TRANSLATOR_XYZ)
				return;
			if (isolateActiveRing && state != candidate)
				return;

			const XMMATRIX mirror = GetMirrorMatrix(candidate, camera);
			const XMMATRIX ringOrientation = axisFrame * mirror * localRotation;
			const XMVECTOR ringNormal = XMVector3Normalize(XMVector3TransformNormal(
				XMVectorSet(1, 0, 0, 0), ringOrientation));
			const XMVECTOR planarView = viewToCamera
				- ringNormal * XMVectorGetX(XMVector3Dot(viewToCamera, ringNormal));
			const bool cullBack = tool_rotation_front_facing_only && !dragging
				&& XMVectorGetX(XMVector3LengthSq(planarView)) > 0.000001f;
			const float ringWidth = circle2_width * tool_thickness;
			const float centerRadius = circle_radius - ringWidth * 0.5f;
			constexpr uint32_t segmentCount = 90;
			const uint32_t tubeSides = tool_rotation_ring_tube_sides >= 3
				? std::clamp(tool_rotation_ring_tube_sides, 3u, 16u) : 0u;
			std::vector<Vertex> vertices;
			vertices.reserve(segmentCount * (tubeSides > 0 ? tubeSides * 6 : 6));

			auto appendVertex = [&](float angle, float tubeAngle) {
				const float radialOffset = tubeSides > 0
					? std::cos(tubeAngle) * ringWidth * 0.5f : 0.0f;
				const float axialOffset = tubeSides > 0
					? std::sin(tubeAngle) * ringWidth * 0.5f : 0.0f;
				const float radius = centerRadius + radialOffset;
				vertices.push_back({ XMFLOAT4(
					axialOffset,
					std::sin(angle) * radius,
					std::cos(angle) * radius,
					1), XMFLOAT4(1, 1, 1, 1) });
			};

			for (uint32_t i = 0; i < segmentCount; ++i)
			{
				const float angle0 = (float)i / (float)segmentCount * XM_2PI;
				const float angle1 = (float)(i + 1) / (float)segmentCount * XM_2PI;
				const float middle = (angle0 + angle1) * 0.5f;
				if (cullBack)
				{
					const XMVECTOR localMidpoint = XMVectorSet(
						0, std::sin(middle) * centerRadius,
						std::cos(middle) * centerRadius, 0);
					const XMVECTOR worldMidpoint = XMVector3TransformNormal(
						localMidpoint, ringOrientation);
					if (XMVectorGetX(XMVector3Dot(worldMidpoint, planarView)) < 0)
						continue;
				}

				if (tubeSides > 0)
				{
					for (uint32_t side = 0; side < tubeSides; ++side)
					{
						const float tube0 = (float)side / (float)tubeSides * XM_2PI;
						const float tube1 = (float)(side + 1) / (float)tubeSides * XM_2PI;
						appendVertex(angle0, tube0);
						appendVertex(angle1, tube0);
						appendVertex(angle0, tube1);
						appendVertex(angle0, tube1);
						appendVertex(angle1, tube0);
						appendVertex(angle1, tube1);
					}
				}
				else
				{
					const float innerRadius = circle_radius - ringWidth;
					vertices.push_back({ XMFLOAT4(0, std::sin(angle0) * innerRadius, std::cos(angle0) * innerRadius, 1), XMFLOAT4(1,1,1,1) });
					vertices.push_back({ XMFLOAT4(0, std::sin(angle1) * innerRadius, std::cos(angle1) * innerRadius, 1), XMFLOAT4(1,1,1,1) });
					vertices.push_back({ XMFLOAT4(0, std::sin(angle0) * circle_radius, std::cos(angle0) * circle_radius, 1), XMFLOAT4(1,1,1,1) });
					vertices.push_back({ XMFLOAT4(0, std::sin(angle0) * circle_radius, std::cos(angle0) * circle_radius, 1), XMFLOAT4(1,1,1,1) });
					vertices.push_back({ XMFLOAT4(0, std::sin(angle1) * circle_radius, std::cos(angle1) * circle_radius, 1), XMFLOAT4(1,1,1,1) });
					vertices.push_back({ XMFLOAT4(0, std::sin(angle1) * innerRadius, std::cos(angle1) * innerRadius, 1), XMFLOAT4(1,1,1,1) });
				}
			}

			if (vertices.empty())
				return;
			const GraphicsDevice::GPUAllocation mem = device->AllocateGPU(
				sizeof(Vertex) * vertices.size(), cmd);
			std::memcpy(mem.data, vertices.data(), sizeof(Vertex) * vertices.size());
			const GPUBuffer* vertexBuffers[] = { &mem.buffer };
			constexpr uint32_t strides[] = { sizeof(Vertex) };
			const uint64_t offsets[] = { mem.offset };
			device->BindVertexBuffers(vertexBuffers, 0, arraysize(vertexBuffers), strides, offsets, cmd);
			XMStoreFloat4x4(&sb.g_xTransform, axisFrame * mirror * mat);
			sb.g_xColor = state == candidate && !dragging ? highlight_color : baseColor;
			sb.g_xColor.w *= tool_opacity;
			device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
			device->Draw((uint32_t)vertices.size(), 0, cmd);
		};

		float darken = 1;
		if (tool_axis_x_enabled)
		{
			darken = isLocalSpace ? 1 : (camera.Eye.x < transform.translation_local.x ? tool_darken_negative_axes : 1);
			drawRing(TRANSLATOR_X, matX, XMFLOAT4(darken, channel_min * darken, channel_min * darken, 1));
		}
		if (tool_axis_y_enabled)
		{
			darken = isLocalSpace ? 1 : (camera.Eye.y < transform.translation_local.y ? tool_darken_negative_axes : 1);
			drawRing(TRANSLATOR_Y, matY, XMFLOAT4(channel_min * darken, darken, channel_min * darken, 1));
		}
		if (tool_axis_z_enabled)
		{
			darken = isLocalSpace ? 1 : (camera.Eye.z < transform.translation_local.z ? tool_darken_negative_axes : 1);
			drawRing(TRANSLATOR_Z, matZ, XMFLOAT4(channel_min * darken, channel_min * darken, darken, 1));
		}
	}
	else
	{
		device->BindPipelineState(&pso_solidpart, cmd);

		uint32_t vertexCount = 0;
		GraphicsDevice::GPUAllocation mem;

		if (isRotator)
		{
			constexpr uint32_t segmentCount = 90;
			constexpr uint32_t circle_triangleCount = segmentCount * 2;
			vertexCount = circle_triangleCount * 3;
			mem = device->AllocateGPU(sizeof(Vertex) * vertexCount, cmd);
			uint8_t* dst = (uint8_t*)mem.data;
			for (uint32_t i = 0; i < segmentCount; ++i)
			{
				const float angle0 = (float)i / (float)segmentCount * XM_2PI;
				const float angle1 = (float)(i + 1) / (float)segmentCount * XM_2PI;

				// circle:
				const float cw = (tool_rotation_ring_tube_sides >= 3
					? circle2_width : circle_width) * tool_thickness;
				const float circle_radius_inner = circle_radius - cw;
				const Vertex verts[] = {
					{XMFLOAT4(0, std::sin(angle0) * circle_radius_inner, std::cos(angle0) * circle_radius_inner, 1), XMFLOAT4(1,1,1,1)},
					{XMFLOAT4(0, std::sin(angle1) * circle_radius_inner, std::cos(angle1) * circle_radius_inner, 1), XMFLOAT4(1,1,1,1)},
					{XMFLOAT4(0, std::sin(angle0) * circle_radius, std::cos(angle0) * circle_radius, 1), XMFLOAT4(1,1,1,1)},
					{XMFLOAT4(0, std::sin(angle0) * circle_radius, std::cos(angle0) * circle_radius, 1), XMFLOAT4(1,1,1,1)},
					{XMFLOAT4(0, std::sin(angle1) * circle_radius, std::cos(angle1) * circle_radius, 1), XMFLOAT4(1,1,1,1)},
					{XMFLOAT4(0, std::sin(angle1) * circle_radius_inner, std::cos(angle1) * circle_radius_inner, 1), XMFLOAT4(1,1,1,1)},
				};
				std::memcpy(dst, verts, sizeof(verts));
				dst += sizeof(verts);
			}
		}
		else
		{
			constexpr uint32_t segmentCount = 18;
			constexpr uint32_t cylinder_triangleCount = segmentCount * 2;
			constexpr uint32_t cone_triangleCount = cylinder_triangleCount;
			if (isTranslator)
			{
				vertexCount = (cylinder_triangleCount + cone_triangleCount) * 3;
			}
			else if (isScalator)
			{
				vertexCount = cylinder_triangleCount * 3 + arraysize(cubeVerts);
			}
			mem = device->AllocateGPU(sizeof(Vertex) * vertexCount, cmd);

			constexpr float cone_length = 0.75f;
			float cylinder_length = axis_length;
			if (isTranslator)
			{
				cylinder_length -= cone_length;
			}
			uint8_t* dst = (uint8_t*)mem.data;
			for (uint32_t i = 0; i < segmentCount; ++i)
			{
				const float angle0 = (float)i / (float)segmentCount * XM_2PI;
				const float angle1 = (float)(i + 1) / (float)segmentCount * XM_2PI;
				// cylinder base:
				{
					const float cylinder_radius = 0.075f * tool_thickness;
					const Vertex verts[] = {
						{XMFLOAT4(origin_size, std::sin(angle0) * cylinder_radius, std::cos(angle0) * cylinder_radius, 1), XMFLOAT4(1,1,1,1)},
						{XMFLOAT4(origin_size, std::sin(angle1) * cylinder_radius, std::cos(angle1) * cylinder_radius, 1), XMFLOAT4(1,1,1,1)},
						{XMFLOAT4(cylinder_length, std::sin(angle0) * cylinder_radius, std::cos(angle0) * cylinder_radius, 1), XMFLOAT4(1,1,1,1)},
						{XMFLOAT4(cylinder_length, std::sin(angle0) * cylinder_radius, std::cos(angle0) * cylinder_radius, 1), XMFLOAT4(1,1,1,1)},
						{XMFLOAT4(cylinder_length, std::sin(angle1) * cylinder_radius, std::cos(angle1) * cylinder_radius, 1), XMFLOAT4(1,1,1,1)},
						{XMFLOAT4(origin_size, std::sin(angle1) * cylinder_radius, std::cos(angle1) * cylinder_radius, 1), XMFLOAT4(1,1,1,1)},
					};
					std::memcpy(dst, verts, sizeof(verts));
					dst += sizeof(verts);
				}
				if (isTranslator)
				{
					// cone cap:
					const float cone_radius = origin_size * tool_thickness;
					const Vertex verts[] = {
						{XMFLOAT4(cylinder_length, 0, 0, 1), XMFLOAT4(1,1,1,1)},
						{XMFLOAT4(cylinder_length, std::sin(angle0) * cone_radius, std::cos(angle0) * cone_radius, 1), XMFLOAT4(1,1,1,1)},
						{XMFLOAT4(cylinder_length, std::sin(angle1) * cone_radius, std::cos(angle1) * cone_radius, 1), XMFLOAT4(1,1,1,1)},
						{XMFLOAT4(axis_length, 0, 0, 1), XMFLOAT4(1,1,1,1)},
						{XMFLOAT4(cylinder_length, std::sin(angle0) * cone_radius, std::cos(angle0) * cone_radius, 1), XMFLOAT4(1,1,1,1)},
						{XMFLOAT4(cylinder_length, std::sin(angle1) * cone_radius, std::cos(angle1) * cone_radius, 1), XMFLOAT4(1,1,1,1)},
					};
					std::memcpy(dst, verts, sizeof(verts));
					dst += sizeof(verts);
				}
			}

			if (isScalator)
			{
				// cube cap:
				for (uint32_t i = 0; i < arraysize(cubeVerts); ++i)
				{
					Vertex vert = cubeVerts[i];
					const float cube_size = origin_size * tool_thickness;
					vert.position.x = vert.position.x * cube_size + cylinder_length - cube_size;
					vert.position.y = vert.position.y * cube_size;
					vert.position.z = vert.position.z * cube_size;
					std::memcpy(dst, &vert, sizeof(vert));
					dst += sizeof(vert);
				}
			}
		}

		const GPUBuffer* vbs[] = {
			&mem.buffer,
		};
		constexpr uint32_t strides[] = {
			sizeof(Vertex),
		};
		const uint64_t offsets[] = {
			mem.offset,
		};
		device->BindVertexBuffers(vbs, 0, arraysize(vbs), strides, offsets, cmd);

		float darken = 1;

		if (!isRotator || !is2D)
		{
			if (tool_axis_x_enabled)
			{
				// x
				XMStoreFloat4x4(&sb.g_xTransform, matX * GetMirrorMatrix(TRANSLATOR_X, camera) * mat);
				darken = isLocalSpace ? 1 : (camera.Eye.x < transform.translation_local.x ? tool_darken_negative_axes : 1);
				sb.g_xColor = state == TRANSLATOR_X ? highlight_color : XMFLOAT4(darken, channel_min * darken, channel_min * darken, 1);
				sb.g_xColor.w *= tool_opacity;
				device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
				device->Draw(vertexCount, 0, cmd);
			}

			if (tool_axis_y_enabled)
			{
				// y
				XMStoreFloat4x4(&sb.g_xTransform, matY * GetMirrorMatrix(TRANSLATOR_Y, camera) * mat);
				darken = isLocalSpace ? 1 : (camera.Eye.y < transform.translation_local.y ? tool_darken_negative_axes : 1);
				sb.g_xColor = state == TRANSLATOR_Y ? highlight_color : XMFLOAT4(channel_min * darken, darken, channel_min * darken, 1);
				sb.g_xColor.w *= tool_opacity;
				device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
				device->Draw(vertexCount, 0, cmd);
			}
		}

		if (tool_axis_z_enabled && (!is2D || isRotator))
		{
			// z
			XMStoreFloat4x4(&sb.g_xTransform, matZ * GetMirrorMatrix(TRANSLATOR_Z, camera) * mat);
			darken = isLocalSpace ? 1 : (camera.Eye.z < transform.translation_local.z ? tool_darken_negative_axes : 1);
			sb.g_xColor = state == TRANSLATOR_Z ? highlight_color : XMFLOAT4(channel_min * darken, channel_min * darken, darken, 1);
			sb.g_xColor.w *= tool_opacity;
			device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
			device->Draw(vertexCount, 0, cmd);
		}

	}

	if (isRotator && !is2D
		&& (!dragging || state == TRANSLATOR_XYZ))
	{
		// Another circle for rotator, a bit thinner and screen facing, so new geo:
		constexpr uint32_t segmentCount = 90;
		constexpr uint32_t circle2_triangleCount = segmentCount * 2;
		uint32_t vertexCount = circle2_triangleCount * 3;
		GraphicsDevice::GPUAllocation mem = device->AllocateGPU(sizeof(Vertex) * vertexCount, cmd);
		uint8_t* dst = (uint8_t*)mem.data;
		for (uint32_t i = 0; i < segmentCount; ++i)
		{
			const float angle0 = (float)i / (float)segmentCount * XM_2PI;
			const float angle1 = (float)(i + 1) / (float)segmentCount * XM_2PI;

			// circle:
			const float c2w = circle2_width * tool_thickness;
			const float circle2_radius_inner = circle2_radius - c2w;
			const Vertex verts[] = {
				{XMFLOAT4(0, std::sin(angle0) * circle2_radius_inner, std::cos(angle0) * circle2_radius_inner, 1), XMFLOAT4(1,1,1,1)},
				{XMFLOAT4(0, std::sin(angle1) * circle2_radius_inner, std::cos(angle1) * circle2_radius_inner, 1), XMFLOAT4(1,1,1,1)},
				{XMFLOAT4(0, std::sin(angle0) * circle2_radius, std::cos(angle0) * circle2_radius, 1), XMFLOAT4(1,1,1,1)},
				{XMFLOAT4(0, std::sin(angle0) * circle2_radius, std::cos(angle0) * circle2_radius, 1), XMFLOAT4(1,1,1,1)},
				{XMFLOAT4(0, std::sin(angle1) * circle2_radius, std::cos(angle1) * circle2_radius, 1), XMFLOAT4(1,1,1,1)},
				{XMFLOAT4(0, std::sin(angle1) * circle2_radius_inner, std::cos(angle1) * circle2_radius_inner, 1), XMFLOAT4(1,1,1,1)},
			};
			std::memcpy(dst, verts, sizeof(verts));
			dst += sizeof(verts);
		}

		const GPUBuffer* vbs[] = {
			&mem.buffer,
		};
		constexpr uint32_t strides[] = {
			sizeof(Vertex),
		};
		const uint64_t offsets[] = {
			mem.offset,
		};
		device->BindVertexBuffers(vbs, 0, arraysize(vbs), strides, offsets, cmd);

		XMMATRIX mat_no_localrot = XMMatrixScaling(dist, dist, dist) * XMMatrixTranslationFromVector(transform.GetPositionV()) * VP;
		XMStoreFloat4x4(&sb.g_xTransform,
			XMMatrixRotationY(XM_PIDIV2) *
			XMMatrixInverse(nullptr, XMMatrixLookToLH(XMVectorZero(), XMVector3Normalize(transform.GetPositionV() - camera.GetEye()), camera.GetUp())) *
			mat_no_localrot
		);
		sb.g_xColor = state == TRANSLATOR_XYZ && !dragging
			? highlight_color : XMFLOAT4(1, 1, 1, dragging ? 1.0f : 0.5f);
		sb.g_xColor.w *= tool_opacity;
		device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
		device->Draw(vertexCount, 0, cmd);
	}

	if (isRotator && tool_trackball_enabled
		&& state == TRANSLATOR_TRACKBALL)
	{
		device->BindPipelineState(&pso_solidpart, cmd);
		constexpr uint32_t segmentCount = 48;
		constexpr uint32_t vertexCount = segmentCount * 3;
		GraphicsDevice::GPUAllocation mem = device->AllocateGPU(
			sizeof(Vertex) * vertexCount, cmd);
		uint8_t* dst = static_cast<uint8_t*>(mem.data);
		for (uint32_t i = 0; i < segmentCount; ++i)
		{
			const float angle0 = static_cast<float>(i) / segmentCount * XM_2PI;
			const float angle1 = static_cast<float>(i + 1) / segmentCount * XM_2PI;
			const Vertex verts[] = {
				{ XMFLOAT4(0, 0, 0, 1), XMFLOAT4(1, 1, 1, 1) },
				{ XMFLOAT4(0, std::cos(angle0) * tool_trackball_radius,
					std::sin(angle0) * tool_trackball_radius, 1), XMFLOAT4(1, 1, 1, 1) },
				{ XMFLOAT4(0, std::cos(angle1) * tool_trackball_radius,
					std::sin(angle1) * tool_trackball_radius, 1), XMFLOAT4(1, 1, 1, 1) },
			};
			std::memcpy(dst, verts, sizeof(verts));
			dst += sizeof(verts);
		}
		const GPUBuffer* vertexBuffers[] = { &mem.buffer };
		constexpr uint32_t strides[] = { sizeof(Vertex) };
		const uint64_t offsets[] = { mem.offset };
		device->BindVertexBuffers(
			vertexBuffers, 0, arraysize(vertexBuffers), strides, offsets, cmd);
		const XMMATRIX matNoLocalRotation = XMMatrixScaling(dist, dist, dist)
			* XMMatrixTranslationFromVector(transform.GetPositionV()) * VP;
		XMStoreFloat4x4(&sb.g_xTransform,
			XMMatrixRotationY(XM_PIDIV2)
			* XMMatrixInverse(nullptr, XMMatrixLookToLH(
				XMVectorZero(), XMVector3Normalize(
					transform.GetPositionV() - camera.GetEye()), camera.GetUp()))
			* matNoLocalRotation);
		sb.g_xColor = XMFLOAT4(1.0f, 0.68f, 0.08f, dragging ? 0.42f : 0.30f);
		sb.g_xColor.w *= tool_opacity;
		device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
		device->Draw(vertexCount, 0, cmd);
	}

	// Origin:
	if(!isRotator)
	{
		device->BindPipelineState(&pso_solidpart, cmd);

		GraphicsDevice::GPUAllocation mem = device->AllocateGPU(sizeof(cubeVerts), cmd);
		std::memcpy(mem.data, cubeVerts, sizeof(cubeVerts));
		const GPUBuffer* vbs[] = {
			&mem.buffer,
		};
		constexpr uint32_t strides[] = {
			sizeof(Vertex),
		};
		const uint64_t offsets[] = {
			mem.offset,
		};
		device->BindVertexBuffers(vbs, 0, arraysize(vbs), strides, offsets, cmd);

		const float os = origin_size * tool_thickness;
		XMStoreFloat4x4(&sb.g_xTransform, XMMatrixScaling(os, os, os) * mat);
		sb.g_xColor = state == TRANSLATOR_XYZ ? highlight_color : XMFLOAT4(0.5f, 0.5f, 0.5f, 1);
		sb.g_xColor.w *= tool_opacity;
		device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
		device->Draw(arraysize(cubeVerts), 0, cmd);
	}

	// Planes:
	if (isTranslator)
	{
		// Wire part:
		{
			device->BindPipelineState(&pso_wirepart, cmd);

			const Vertex verts[] = {
				{XMFLOAT4(plane_min,plane_min,0,1), XMFLOAT4(1,1,1,1)},
				{XMFLOAT4(plane_min,plane_max,0,1), XMFLOAT4(1,1,1,1)},

				{XMFLOAT4(plane_min,plane_max,0,1), XMFLOAT4(1,1,1,1)},
				{XMFLOAT4(plane_max,plane_max,0,1), XMFLOAT4(1,1,1,1)},

				{XMFLOAT4(plane_max,plane_max,0,1), XMFLOAT4(1,1,1,1)},
				{XMFLOAT4(plane_max,plane_min,0,1), XMFLOAT4(1,1,1,1)},

				{XMFLOAT4(plane_max,plane_min,0,1), XMFLOAT4(1,1,1,1)},
				{XMFLOAT4(plane_min,plane_min,0,1), XMFLOAT4(1,1,1,1)},
			};
			GraphicsDevice::GPUAllocation mem = device->AllocateGPU(sizeof(verts), cmd);
			std::memcpy(mem.data, verts, sizeof(verts));
			const GPUBuffer* vbs[] = {
				&mem.buffer,
			};
			constexpr uint32_t strides[] = {
				sizeof(Vertex),
			};
			const uint64_t offsets[] = {
				mem.offset,
			};
			device->BindVertexBuffers(vbs, 0, arraysize(vbs), strides, offsets, cmd);

			if (tool_axis_x_enabled && tool_axis_y_enabled)
			{
				// xy
				XMStoreFloat4x4(&sb.g_xTransform, matX * GetMirrorMatrix(TRANSLATOR_XY, camera) * mat);
				sb.g_xColor = state == TRANSLATOR_XY ? highlight_color : XMFLOAT4(channel_min, channel_min, 1, 1);
				sb.g_xColor.w *= tool_opacity;
				device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
				device->Draw(arraysize(verts), 0, cmd);
			}

			if (!is2D)
			{
				if (tool_axis_x_enabled && tool_axis_z_enabled)
				{
					// xz
					XMStoreFloat4x4(&sb.g_xTransform, matZ * GetMirrorMatrix(TRANSLATOR_XZ, camera) * mat);
					sb.g_xColor = state == TRANSLATOR_XZ ? highlight_color : XMFLOAT4(channel_min, 1, channel_min, 1);
					sb.g_xColor.w *= tool_opacity;
					device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
					device->Draw(arraysize(verts), 0, cmd);
				}

				if (tool_axis_y_enabled && tool_axis_z_enabled)
				{
					// yz
					XMStoreFloat4x4(&sb.g_xTransform, matY * GetMirrorMatrix(TRANSLATOR_YZ, camera) * mat);
					sb.g_xColor = state == TRANSLATOR_YZ ? highlight_color : XMFLOAT4(1, channel_min, channel_min, 1);
					sb.g_xColor.w *= tool_opacity;
					device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
					device->Draw(arraysize(verts), 0, cmd);
				}
			}
		}

		// Quad part:
		{
			device->BindPipelineState(&pso_solidpart, cmd);

			const Vertex verts[] = {
				{XMFLOAT4(plane_min,plane_min,0,1), XMFLOAT4(1,1,1,1)},
				{XMFLOAT4(plane_max,plane_min,0,1), XMFLOAT4(1,1,1,1)},
				{XMFLOAT4(plane_max,plane_max,0,1), XMFLOAT4(1,1,1,1)},

				{XMFLOAT4(plane_min,plane_min,0,1), XMFLOAT4(1,1,1,1)},
				{XMFLOAT4(plane_max,plane_max,0,1), XMFLOAT4(1,1,1,1)},
				{XMFLOAT4(plane_min,plane_max,0,1), XMFLOAT4(1,1,1,1)},
			};
			GraphicsDevice::GPUAllocation mem = device->AllocateGPU(sizeof(verts), cmd);
			std::memcpy(mem.data, verts, sizeof(verts));
			const GPUBuffer* vbs[] = {
				&mem.buffer,
			};
			constexpr uint32_t strides[] = {
				sizeof(Vertex),
			};
			const uint64_t offsets[] = {
				mem.offset,
			};
			device->BindVertexBuffers(vbs, 0, arraysize(vbs), strides, offsets, cmd);

			if (tool_axis_x_enabled && tool_axis_y_enabled)
			{
				// xy
				XMStoreFloat4x4(&sb.g_xTransform, matX * GetMirrorMatrix(TRANSLATOR_XY, camera) * mat);
				sb.g_xColor = state == TRANSLATOR_XY ? highlight_color : XMFLOAT4(channel_min, channel_min, 1, 0.4f);
				sb.g_xColor.w *= tool_opacity;
				device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
				device->Draw(arraysize(verts), 0, cmd);
			}

			if (!is2D)
			{
				if (tool_axis_x_enabled && tool_axis_z_enabled)
				{
					// xz
					XMStoreFloat4x4(&sb.g_xTransform, matZ * GetMirrorMatrix(TRANSLATOR_XZ, camera) * mat);
					sb.g_xColor = state == TRANSLATOR_XZ ? highlight_color : XMFLOAT4(channel_min, 1, channel_min, 0.4f);
					sb.g_xColor.w *= tool_opacity;
					device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
					device->Draw(arraysize(verts), 0, cmd);
				}

				if (tool_axis_y_enabled && tool_axis_z_enabled)
				{
					// yz
					XMStoreFloat4x4(&sb.g_xTransform, matY * GetMirrorMatrix(TRANSLATOR_YZ, camera) * mat);
					sb.g_xColor = state == TRANSLATOR_YZ ? highlight_color : XMFLOAT4(1, channel_min, channel_min, 0.4f);
					sb.g_xColor.w *= tool_opacity;
					device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
					device->Draw(arraysize(verts), 0, cmd);
				}
			}
		}
	}


	// Axis texts:
	if (!isRotator && (!isLocalSpace || tool_axis_text_in_local_space))
	{
		char TEXT[3];
		XMMATRIX R = XMLoadFloat3x3(&camera.rotationMatrix);
		const XMMATRIX axisRotation = GetLocalRotation();

		wi::font::Params params;
		params.v_align = wi::font::WIFALIGN_CENTER;
		params.h_align = wi::font::WIFALIGN_CENTER;
		params.scaling = 0.04f * dist;
		params.customProjection = &VP;
		params.customRotation = &R;
		params.shadowColor = wi::Color(0, 0, 0, uint8_t(127 * tool_opacity));
		params.shadow_softness = 0.8f;
		XMVECTOR pos = transform.GetPositionV();

		float darken = 1;
		if (tool_axis_x_enabled)
		{
			darken = camera.Eye.x < transform.translation_local.x ? tool_darken_negative_axes : 1;
			params.color = wi::Color::fromFloat4(XMFLOAT4(darken, channel_min * darken, channel_min * darken, tool_opacity));
			XMStoreFloat3(&params.position, pos + XMVector3Transform(
				XMVectorSet(axis_length + 0.5f, 0, 0, 0) * dist,
				GetMirrorMatrix(TRANSLATOR_X, camera) * axisRotation));
			std::memset(TEXT, 0, sizeof(TEXT));
			WriteAxisText(TRANSLATOR_X, camera, TEXT);
			wi::font::Draw(TEXT, strlen(TEXT), params, cmd);
		}

		if (tool_axis_y_enabled)
		{
			darken = camera.Eye.y < transform.translation_local.y ? tool_darken_negative_axes : 1;
			params.color = wi::Color::fromFloat4(XMFLOAT4(channel_min * darken, darken, channel_min * darken, tool_opacity));
			XMStoreFloat3(&params.position, pos + XMVector3Transform(
				XMVectorSet(0, axis_length + 0.5f, 0, 0) * dist,
				GetMirrorMatrix(TRANSLATOR_Y, camera) * axisRotation));
			std::memset(TEXT, 0, sizeof(TEXT));
			WriteAxisText(TRANSLATOR_Y, camera, TEXT);
			wi::font::Draw(TEXT, strlen(TEXT), params, cmd);
		}

		if (tool_axis_z_enabled && !is2D)
		{
			darken = camera.Eye.z < transform.translation_local.z ? tool_darken_negative_axes : 1;
			params.color = wi::Color::fromFloat4(XMFLOAT4(channel_min * darken, channel_min * darken, darken, tool_opacity));
			XMStoreFloat3(&params.position, pos + XMVector3Transform(
				XMVectorSet(0, 0, axis_length + 0.5f, 0) * dist,
				GetMirrorMatrix(TRANSLATOR_Z, camera) * axisRotation));
			std::memset(TEXT, 0, sizeof(TEXT));
			WriteAxisText(TRANSLATOR_Z, camera, TEXT);
			wi::font::Draw(TEXT, strlen(TEXT), params, cmd);
		}
	}


	// Dragging visualizer:
	if (dragging)
	{
		if (isTranslator)
		{
			// Origin circle:
			{
				device->BindPipelineState(&pso_solidpart, cmd);

				constexpr uint32_t segmentCount = 36;
				constexpr uint32_t vertexCount = segmentCount * 3;
				GraphicsDevice::GPUAllocation mem = device->AllocateGPU(sizeof(Vertex) * vertexCount, cmd);

				uint8_t* dst = (uint8_t*)mem.data;
				for (uint32_t i = 0; i < segmentCount; ++i)
				{
					const float angle0 = (float)i / (float)segmentCount * XM_2PI;
					const float angle1 = (float)(i + 1) / (float)segmentCount * XM_2PI;

					const float radius = 0.2f * dist;
					const Vertex verts[] = {
						{XMFLOAT4(0, 0, 0, 1), XMFLOAT4(1,1,1,1)},
						{XMFLOAT4(0, std::cos(angle0) * radius, std::sin(angle0) * radius, 1), XMFLOAT4(1,1,1,1)},
						{XMFLOAT4(0, std::cos(angle1) * radius, std::sin(angle1) * radius, 1), XMFLOAT4(1,1,1,1)},
					};
					std::memcpy(dst, verts, sizeof(verts));
					dst += sizeof(verts);
				}

				const GPUBuffer* vbs[] = {
					&mem.buffer,
				};
				constexpr uint32_t strides[] = {
					sizeof(Vertex),
				};
				const uint64_t offsets[] = {
					mem.offset,
				};
				device->BindVertexBuffers(vbs, 0, arraysize(vbs), strides, offsets, cmd);

				XMStoreFloat4x4(&sb.g_xTransform,
					XMMatrixRotationY(XM_PIDIV2)*
					XMMatrixInverse(nullptr, XMMatrixLookToLH(XMVectorZero(), XMVector3Normalize(transform_start.GetPositionV() - camera.GetEye()), camera.GetUp()))*
					XMMatrixTranslationFromVector(transform_start.GetPositionV())*
					VP
				);
				sb.g_xColor = XMFLOAT4(1, 1, 1, 0.5f);
				sb.g_xColor.w *= tool_opacity;
				device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
				device->Draw(vertexCount, 0, cmd);
			}

			// Line:
			{
				device->BindPipelineState(&pso_wirepart, cmd);
				const Vertex verts[] = {
					{XMFLOAT4(transform_start.translation_local.x, transform_start.translation_local.y, transform_start.translation_local.z, 1), XMFLOAT4(1,1,1,1)},
					{XMFLOAT4(transform.translation_local.x, transform.translation_local.y, transform.translation_local.z, 1), XMFLOAT4(1,1,1,1)},
				};
				GraphicsDevice::GPUAllocation mem = device->AllocateGPU(sizeof(verts), cmd);
				std::memcpy(mem.data, verts, sizeof(verts));
				const GPUBuffer* vbs[] = {
					&mem.buffer,
				};
				constexpr uint32_t strides[] = {
					sizeof(Vertex),
				};
				const uint64_t offsets[] = {
					mem.offset,
				};
				device->BindVertexBuffers(vbs, 0, arraysize(vbs), strides, offsets, cmd);

				XMStoreFloat4x4(&sb.g_xTransform, VP);
				sb.g_xColor = XMFLOAT4(1, 1, 1, 1);
				sb.g_xColor.w *= tool_opacity;
				device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
				device->Draw(arraysize(verts), 0, cmd);
			}
		}
		else if (isRotator && state != TRANSLATOR_TRACKBALL)
		{
			device->BindPipelineState(&pso_solidpart, cmd);

			constexpr uint32_t segmentCount = 90;
			constexpr uint32_t vertexCount = segmentCount * 3;
			GraphicsDevice::GPUAllocation mem = device->AllocateGPU(sizeof(Vertex) * vertexCount, cmd);

			switch (state)
			{
			case Translator::TRANSLATOR_X:
				XMStoreFloat4x4(&sb.g_xTransform, matX * mat);
				break;
			case Translator::TRANSLATOR_Y:
				XMStoreFloat4x4(&sb.g_xTransform, matY * mat);
				break;
			case Translator::TRANSLATOR_Z:
				XMStoreFloat4x4(&sb.g_xTransform, matZ * mat);
				break;
			case Translator::TRANSLATOR_XYZ:
				XMStoreFloat4x4(&sb.g_xTransform,
					XMMatrixRotationY(XM_PIDIV2) *
					XMMatrixInverse(nullptr, XMMatrixLookToLH(XMVectorZero(), XMVector3Normalize(transform.GetPositionV() - camera.GetEye()), camera.GetUp())) *
					mat
				);
				break;
			default:
				break;
			}

			uint8_t* dst = (uint8_t*)mem.data;
			for (uint32_t i = 0; i < segmentCount; ++i)
			{
				const float angle0 = (float)i / (float)segmentCount * angle + angle_start;
				const float angle1 = (float)(i + 1) / (float)segmentCount * angle + angle_start;

				const float ringWidth = circle2_width * tool_thickness;
				const float radius = state == TRANSLATOR_XYZ
					? circle2_radius - ringWidth : circle_radius - ringWidth;
				const Vertex verts[] = {
					{XMFLOAT4(0, 0, 0, 1), XMFLOAT4(1,1,1,1)},
					{XMFLOAT4(0, std::cos(angle0) * radius, std::sin(angle0) * radius, 1), XMFLOAT4(1,1,1,1)},
					{XMFLOAT4(0, std::cos(angle1) * radius, std::sin(angle1) * radius, 1), XMFLOAT4(1,1,1,1)},
				};
				std::memcpy(dst, verts, sizeof(verts));
				dst += sizeof(verts);
			}

			const GPUBuffer* vbs[] = {
				&mem.buffer,
			};
			const uint32_t strides[] = {
				sizeof(Vertex),
			};
			const uint64_t offsets[] = {
				mem.offset,
			};
			device->BindVertexBuffers(vbs, 0, arraysize(vbs), strides, offsets, cmd);

			sb.g_xColor = XMFLOAT4(1, 1, 1, 0.5f);
			sb.g_xColor.w *= tool_opacity;
			device->BindDynamicConstantBuffer(sb, CBSLOT_RENDERER_MISC, cmd);
			device->Draw(vertexCount, 0, cmd);
		}

		wi::font::Params params;
		params.posX = currentMouse.x - 20;
		params.posY = currentMouse.y + 20;
		params.v_align = wi::font::WIFALIGN_TOP;
		params.h_align = wi::font::WIFALIGN_RIGHT;
		params.scaling = 0.8f;
		params.shadowColor = wi::Color::Black();

		char text[256] = {};

		if (isTranslator)
		{
			snprintf(text, arraysize(text), "Offset = %.2f, %.2f, %.2f", transform.translation_local.x - transform_start.translation_local.x, transform.translation_local.y - transform_start.translation_local.y, transform.translation_local.z - transform_start.translation_local.z);
		}
		if (isRotator)
		{
			switch (state)
			{
			case Translator::TRANSLATOR_X:
				snprintf(text, arraysize(text), "Axis = X\nAngle = %.2f degrees", wi::math::RadiansToDegrees(angle));
				break;
			case Translator::TRANSLATOR_Y:
				snprintf(text, arraysize(text), "Axis = Y\nAngle = %.2f degrees", wi::math::RadiansToDegrees(angle));
				break;
			case Translator::TRANSLATOR_Z:
				snprintf(text, arraysize(text), "Axis = Z\nAngle = %.2f degrees", wi::math::RadiansToDegrees(angle));
				break;
			case Translator::TRANSLATOR_XYZ:
				snprintf(text, arraysize(text), "Axis = Screen\nAngle = %.2f degrees", wi::math::RadiansToDegrees(angle));
				break;
			default:
				break;
			}
		}
		if (isScalator)
		{
			snprintf(text, arraysize(text), "Scaling = %.2f, %.2f, %.2f", transform.scale_local.x / transform_start.scale_local.x, transform.scale_local.y / transform_start.scale_local.y, transform.scale_local.z / transform_start.scale_local.z);
		}
		params.shadowColor.setA(uint8_t(tool_opacity * 255.0f));
		params.color.setA(uint8_t(tool_opacity * 255.0f));
		wi::font::Draw(text, params, cmd);
	}

	device->EventEnd(cmd);
}

void Translator::PreTranslate()
{
	const Scene& scene = *this->scene;

	if (!dragging)
	{
		transform.ClearTransform();
	}
	has_selected_transform = false;

	// Find the center of all the entities that are selected:
	XMVECTOR centerV = XMVectorSet(0, 0, 0, 0);
	float count = 0;
	for (auto& x : selected)
	{
		TransformComponent* transform = scene.transforms.GetComponent(x.entity);
		if (transform != nullptr)
		{
			transform->UpdateTransform();
			centerV = XMVectorAdd(centerV, transform->GetPositionV());
			count += 1.0f;
			has_selected_transform = true;
		}
	}

	if (!has_selected_transform)
		return;

	// Offset translator to center position and perform attachments:
	if (count > 0)
	{
		centerV /= count;
		XMStoreFloat3(&transform.translation_local, centerV);
		transform.SetDirty();
		transform.UpdateTransform();
	}

	// translator "bind matrix"
	const XMMATRIX B = XMMatrixInverse(nullptr, XMLoadFloat4x4(&transform.world));

	matrices_current.clear();
	for (const auto& x : selectedEntitiesNonRecursive)
	{
		const TransformComponent* transform_selected = scene.transforms.GetComponent(x);
		if (transform_selected != nullptr)
		{
			XMFLOAT4X4 m;
			XMStoreFloat4x4(&m, XMLoadFloat4x4(&transform_selected->world) * B);
			matrices_current.push_back(m);
		}
	}
}
void Translator::PostTranslate() const
{
	const Scene& scene = *this->scene;

	int i = 0;
	for (auto& x : selectedEntitiesNonRecursive)
	{
		TransformComponent* transform_selected = scene.transforms.GetComponent(x);
		if (transform_selected != nullptr)
		{
			const XMFLOAT4X4& m = matrices_current[i++];

			XMMATRIX W = XMLoadFloat4x4(&m);
			const XMMATRIX W_parent = XMLoadFloat4x4(&transform.world);
			W = W * W_parent;
			XMStoreFloat4x4(&transform_selected->world, W);

			// selected to world space:
			transform_selected->ApplyTransform();

			// selected to parent local space (if has parent):
			const HierarchyComponent* hier = scene.hierarchy.GetComponent(x);
			if (hier != nullptr)
			{
				const TransformComponent* transform_parent = scene.transforms.GetComponent(hier->parentID);
				if (transform_parent != nullptr)
				{
					transform_selected->MatrixTransform(XMMatrixInverse(nullptr, XMLoadFloat4x4(&transform_parent->world)));
				}
			}
		}
	}
}

XMMATRIX Translator::GetMirrorMatrix(TRANSLATOR_STATE state, const CameraComponent& camera) const
{
	XMMATRIX mirror = XMMatrixIdentity();

	if (isRotator || isLocalSpace)
		return mirror;

	mirror *= XMMatrixScaling(
		tool_world_axis_sign.x,
		tool_world_axis_sign.y,
		tool_world_axis_sign.z);

	if (!tool_mirror_axes_to_camera)
		return mirror;

	switch (state)
	{
	case Translator::TRANSLATOR_X:
		if (camera.Eye.x < transform.translation_local.x)
		{
			mirror *= XMMatrixScaling(-1, 1, 1);
		}
		break;
	case Translator::TRANSLATOR_Y:
		if (camera.Eye.y < transform.translation_local.y)
		{
			mirror *= XMMatrixScaling(1, -1, 1);
		}
		break;
	case Translator::TRANSLATOR_Z:
		if (camera.Eye.z < transform.translation_local.z)
		{
			mirror *= XMMatrixScaling(1, 1, -1);
		}
		break;
	case Translator::TRANSLATOR_XY:
		if (camera.Eye.x < transform.translation_local.x)
		{
			mirror *= XMMatrixScaling(-1, 1, 1);
		}
		if (camera.Eye.y < transform.translation_local.y)
		{
			mirror *= XMMatrixScaling(1, -1, 1);
		}
		break;
	case Translator::TRANSLATOR_XZ:
		if (camera.Eye.x < transform.translation_local.x)
		{
			mirror *= XMMatrixScaling(-1, 1, 1);
		}
		if (camera.Eye.z < transform.translation_local.z)
		{
			mirror *= XMMatrixScaling(1, 1, -1);
		}
		break;
	case Translator::TRANSLATOR_YZ:
		if (camera.Eye.y < transform.translation_local.y)
		{
			mirror *= XMMatrixScaling(1, -1, 1);
		}
		if (camera.Eye.z < transform.translation_local.z)
		{
			mirror *= XMMatrixScaling(1, 1, -1);
		}
		break;
	default:
		break;
	}

	return mirror;
}
void Translator::WriteAxisText(TRANSLATOR_STATE axis, const wi::scene::CameraComponent& camera, char* text) const
{
	if (!tool_mirror_axes_to_camera)
	{
		switch (axis)
		{
		case Translator::TRANSLATOR_X:
			std::memcpy(text, "X", 1);
			break;
		case Translator::TRANSLATOR_Y:
			std::memcpy(text, "Y", 1);
			break;
		case Translator::TRANSLATOR_Z:
			std::memcpy(text, "Z", 1);
			break;
		default:
			break;
		}
		return;
	}
	switch (axis)
	{
	case Translator::TRANSLATOR_X:
		if (camera.Eye.x < transform.translation_local.x)
		{
			std::memcpy(text, "-X", 2);
		}
		else
		{
			std::memcpy(text, "X", 1);
		}
		break;
	case Translator::TRANSLATOR_Y:
		if (camera.Eye.y < transform.translation_local.y)
		{
			std::memcpy(text, "-Y", 2);
		}
		else
		{
			std::memcpy(text, "Y", 1);
		}
		break;
	case Translator::TRANSLATOR_Z:
		if (camera.Eye.z < transform.translation_local.z)
		{
			std::memcpy(text, "-Z", 2);
		}
		else
		{
			std::memcpy(text, "Z", 1);
		}
		break;
	default:
		break;
	}
}

XMMATRIX Translator::GetLocalRotation() const
{
	if (!isLocalSpace || selected.empty())
		return XMMatrixIdentity();

	// Get the rotation from the first selected entity with a transform
	for (const auto& x : selected)
	{
		const TransformComponent* transform_selected = scene->transforms.GetComponent(x.entity);
		if (transform_selected != nullptr)
		{
			XMMATRIX rotation = XMMatrixRotationQuaternion(XMLoadFloat4(&transform_selected->rotation_local));

			const HierarchyComponent* hierarchy_component = scene->hierarchy.GetComponent(x.entity);
			Entity parent = hierarchy_component != nullptr ? hierarchy_component->parentID : INVALID_ENTITY;
			while (parent != INVALID_ENTITY)
			{
				const TransformComponent* parent_transform = scene->transforms.GetComponent(parent);
				if (parent_transform != nullptr)
				{
					rotation = rotation * XMMatrixRotationQuaternion(XMLoadFloat4(&parent_transform->rotation_local));
				}
				const HierarchyComponent* parent_hierarchy = scene->hierarchy.GetComponent(parent);
				parent = parent_hierarchy != nullptr ? parent_hierarchy->parentID : INVALID_ENTITY;
			}

			return rotation;
		}
	}
	return XMMatrixIdentity();
}
