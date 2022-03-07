#include <stdlib.h>
#include <time.h>
#include <algorithm>

#include <imgui.h>

#include "Rigidbody.h"

#define max_rigidbodies 1024
#define physics_timestep 1.0f / 60.0f /* 60 FPS */
#define default_gravity -10.0f
#define default_collision_iterations 8

static CollisionData circle_vs_circle(Rigidbody* b, Rigidbody* a) {
	assert(a && b);

	CollisionData result;

	if (glm::length(a->position - b->position)
		> a->shape.circle.radius + b->shape.circle.radius) {
		result.depth = 0.0f;
		return result;
	}

	result.normal = glm::normalize(a->position - b->position);
	
	result.depth = fabs(glm::length(a->position - b->position) - (a->shape.circle.radius + b->shape.circle.radius));

	result.a = a;
	result.b = b;

	return result;
}

static CollisionData aabb_vs_aabb(Rigidbody* b, Rigidbody* a) {
	assert(a && b);

	glm::vec2 normal = a->position - b->position;

	CollisionData result = {};

	auto a_min = a->position - a->shape.aabb.size * 0.5f;
	auto a_max = a->position + a->shape.aabb.size * 0.5f;
	auto b_min = b->position - b->shape.aabb.size * 0.5f;
	auto b_max = b->position + b->shape.aabb.size * 0.5f;

	float a_ext = (a_max.x - a_min.x) / 2.0f;
	float b_ext = (b_max.x - b_min.x) / 2.0f;

	float x_overlap = a_ext + b_ext - fabs(normal.x);

	if (x_overlap > 0.0f) {
		a_ext = (a_max.y - a_min.y) / 2.0f;
		b_ext = (b_max.y - b_min.y) / 2.0f;

		float y_overlap = a_ext + b_ext - fabs(normal.y);

		if (y_overlap > 0.0f) {
			if (x_overlap < y_overlap) {
				if (normal.x < 0.0f) {
					result.normal = glm::vec2(-1.0f, 0.0f);
				} else {	
					result.normal = glm::vec2(1.0f, 0.0f);
				}
				result.depth = x_overlap;
			} else {
				if (normal.y < 0.0f) {
					result.normal = glm::vec2(0.0f, -1.0f);
				} else {
					result.normal = glm::vec2(0.0f, 1.0f);
				}
				result.depth = y_overlap;
			}
		}
	}

	result.a = a;
	result.b = b;

	return result;
}

static CollisionData circle_vs_aabb(Rigidbody* a, Rigidbody* b) {
	glm::vec2 clamped_pos = a->position;

	/* This becomes fucked if the circle is more than just 
	 * a little bit inside the AABB. */

	auto b_min = b->position - b->shape.aabb.size * 0.5f;
	auto b_max = b->position + b->shape.aabb.size * 0.5f;
	
	if (clamped_pos.x < b_min.x) { clamped_pos.x = b_min.x; }
	if (clamped_pos.y < b_min.y) { clamped_pos.y = b_min.y; }
	if (clamped_pos.x > b_max.x) { clamped_pos.x = b_max.x; }
	if (clamped_pos.y > b_max.y) { clamped_pos.y = b_max.y; }

	CollisionData result = {};

	auto circle_to_clamped = clamped_pos - a->position;
	float dist = glm::length(circle_to_clamped);
	result.depth = a->shape.circle.radius - dist;
	result.normal = circle_to_clamped / dist;

	result.a = a;
	result.b = b;

	return result;
}

static CollisionData aabb_vs_circle(Rigidbody* a, Rigidbody* b) {
	auto cd = circle_vs_aabb(b, a);

	cd.normal = -cd.normal;

	return cd;
}

static float point_vs_plane(Rigidbody* plane, glm::vec2 point) {
	return glm::dot(point, plane->shape.plane.normal) - plane->position.x;
}

static CollisionData plane_vs_aabb(Rigidbody* a, Rigidbody* b) {
	float results[4];

	auto min = b->position - b->shape.aabb.size * 0.5f;
	auto max = b->position + b->shape.aabb.size * 0.5f;

	results[0] = point_vs_plane(a, min);
	results[1] = point_vs_plane(a, max);
	results[2] = point_vs_plane(a, glm::vec2(min.x, max.y));
	results[3] = point_vs_plane(a, glm::vec2(max.x, max.y));

	std::sort(std::begin(results), std::end(results));

	CollisionData result = {};
	result.normal = a->shape.plane.normal;
	result.depth = -results[0];

	result.a = a;
	result.b = b;

	return result;
}

static CollisionData aabb_vs_plane(Rigidbody* a, Rigidbody* b) {
	return plane_vs_aabb(b, a);
}

static CollisionData plane_vs_circle(Rigidbody* a, Rigidbody* b) {
	CollisionData result = {};

	result.normal = a->shape.plane.normal;
	float c_to_p = glm::dot(b->position, a->shape.plane.normal) - a->position.x;
	result.depth = b->shape.circle.radius - c_to_p;

	result.a = a;
	result.b = b;

	return result;
}

static CollisionData circle_vs_plane(Rigidbody* a, Rigidbody* b) {
	return plane_vs_circle(b, a);
}

void Rigidbody::add_force(glm::vec2 force) {
	if (mass <= 0.0f) { return; }

	auto accel = force / mass;
	velocity += accel;
}

void Rigidbody::update(float ts) {
	if (type == plane) {
		/* Forces cannot be applied to planes. */
		return;
	}

	position += velocity * ts;
	add_force(glm::vec2(0.0f, sim->gravity * mass * ts));
}

RigidbodySim::RigidbodySim() : GameBase(), accum(0.0f), gravity(default_gravity), collision_iterations(default_collision_iterations) {
	detectors[Rigidbody::circle][Rigidbody::circle]   = circle_vs_circle;
	detectors[Rigidbody::aabb]  [Rigidbody::aabb]     = aabb_vs_aabb;
	detectors[Rigidbody::circle][Rigidbody::aabb]     = circle_vs_aabb;
	detectors[Rigidbody::aabb]  [Rigidbody::circle]   = aabb_vs_circle;
	detectors[Rigidbody::plane] [Rigidbody::circle]   = plane_vs_circle;
	detectors[Rigidbody::circle][Rigidbody::plane]    = circle_vs_plane;
	detectors[Rigidbody::plane] [Rigidbody::aabb]     = plane_vs_aabb;
	detectors[Rigidbody::aabb]  [Rigidbody::plane]    = aabb_vs_plane;

	rigidbodies = new Rigidbody[max_rigidbodies];
	rigidbody_count = 0;

	auto floor = new_plane( { 0, 1 } );
	floor->position.x = -10.0f;
	floor->mass = 0.0f;
	floor->restitution = 0.1f;

	auto right_side = new_plane( { -1, 0 } );
	right_side->position.x = -10.0f;
	right_side->mass = 0.0f;
	right_side->restitution = 0.1f;

	auto left_side = new_plane( { 1, 0 } );
	left_side->position.x = -10.0f;
	left_side->mass = 0.0f;
	left_side->restitution = 0.1f;

	GameBase::Zoom(0.5f);
}

RigidbodySim::~RigidbodySim() {
	delete[] rigidbodies;
}

void RigidbodySim::Update() {
	accum += deltaTime;

	while (accum >= physics_timestep) {
		for (size_t i = 0; i < rigidbody_count; i++) {
			auto rb = rigidbodies + i;
			rb->update(physics_timestep);
		}

		std::vector<std::pair<Rigidbody*, Rigidbody*>> overlapping;

		for (size_t c = 0; c < collision_iterations; c++) {
			for (size_t i = 0; i < rigidbody_count; i++) {
				auto a = rigidbodies + i;
				for (size_t j = i + 1; j < rigidbody_count; j++) {
					auto b = rigidbodies + j;

					/* No point checking two static bodies or two planes. */
					if (a->mass <= 0.0f && b->mass <= 0.0f) { continue; }
					if (a->type == Rigidbody::plane && b->type == Rigidbody::plane) { continue; }

					CollisionData cd = detectors[a->type][b->type](a, b);
					if (cd.depth > 0.0f) {
						glm::vec2 r_vel = b->velocity - a->velocity;

						float a_inv_mass = a->mass <= 0.0f ? 0.0f : 1.0f / a->mass;
						float b_inv_mass = b->mass <= 0.0f ? 0.0f : 1.0f / b->mass;

						/* Positionally resolve the collision, to prevent sinking
						 * when multiple objects are stacked on each other. */
						glm::vec2 correction = (b_inv_mass / (a_inv_mass + b_inv_mass)) * cd.normal * cd.depth;
						a->position -= a_inv_mass * correction;
						b->position += b_inv_mass * correction;

						/* Resolve the collision */
						float r = (a->restitution + b->restitution) / 2.0f;
						float j = glm::dot(-(1 + r) * (r_vel), cd.normal) / (a_inv_mass + b_inv_mass);

						glm::vec2 force = cd.normal * j;

						a->add_force(-force);
						b->add_force(force);

						/* Calculate and apply a friction impulse. */
						auto o_r_vel = r_vel;
						r_vel = b->velocity - a->velocity;

						glm::vec2 t = glm::normalize(r_vel - glm::dot(o_r_vel, cd.normal) * cd.normal);
						float t_j = glm::dot(-(1 + r) * (r_vel), t) / (a_inv_mass + b_inv_mass);

						/* Average of both bodies' friction values. */
						float stat_fric = (a->stat_friction + b->stat_friction) / 2.0f;

						glm::vec2 fric_imp;
						if (std::fabs(t_j) < j * stat_fric) {
							fric_imp = t_j * t;
						}
						else {
							float kin_fric = (a->kin_friction + b->kin_friction) / 2.0f;
							fric_imp = -j * t * kin_fric;
						}

						a->add_force(-fric_imp);
						b->add_force(fric_imp);
					}
				}
			}
		}

		accum -= physics_timestep;
	}

	GameBase::Update();
}

void RigidbodySim::Render() {
	for (size_t i = 0; i < rigidbody_count; i++) {
		auto rb = rigidbodies + i;

		switch (rb->type) {
			case Rigidbody::circle:
				lines.DrawCircle(rb->position, rb->shape.circle.radius, rb->color);
				break;
			case Rigidbody::aabb: {
				auto min = rb->position - rb->shape.aabb.size * 0.5f;
				auto max = rb->position + rb->shape.aabb.size * 0.5f;

				lines.DrawLineSegment({ min.x, min.y }, { min.x, max.y }, rb->color);
				lines.DrawLineSegment({ min.x, max.y }, { max.x, max.y }, rb->color);
				lines.DrawLineSegment({ max.x, max.y }, { max.x, min.y }, rb->color);
				lines.DrawLineSegment({ max.x, min.y }, { min.x, min.y }, rb->color);
			} break;
			case Rigidbody::plane: {
				float segment = 1000.0f;
				glm::vec2 centre = rb->shape.plane.normal * rb->position.x;
				glm::vec2 tangent(-rb->shape.plane.normal.y, rb->shape.plane.normal.x);
				glm::vec2 start  = centre + (tangent * segment);
				glm::vec2 end    = centre - (tangent * segment);

				lines.DrawLineSegment(start, end, rb->color);
			} break;
		}
	}

	ImGui::Begin("Config");

	ImGui::Text("Bodies: %d", rigidbody_count);
	ImGui::Text("FPS: %g", 1.0 / et);

	ImGui::InputInt("Collision Iterations", (int*)&collision_iterations);
	if (ImGui::IsItemHovered()) {
		ImGui::BeginTooltip();
			ImGui::Text("Higher numbers make the simulation more stable, but also slower.");
		ImGui::EndTooltip();
	}

	ImGui::DragFloat("Gravity", &gravity, 0.01f);

	ImGui::End();

	GameBase::Clear();
	GameBase::Render();
}

void RigidbodySim::OnMouseClick(int mouseButton) {
	if (ImGui::IsAnyItemHovered() || ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow)) { return; }

	if (mouseButton == GLFW_MOUSE_BUTTON_LEFT) {
		auto rb = new_circle(1.0f);
		rb->position = cursorPos;
	} else if (mouseButton == GLFW_MOUSE_BUTTON_RIGHT) {
		auto rb = new_aabb({ 2.0f, 2.0f });
		rb->position = cursorPos;
	}
}

void RigidbodySim::OnMouseRelease(int mouseButton) {
}

Rigidbody* RigidbodySim::new_rigidbody() {
	assert(rigidbody_count < max_rigidbodies && "Too many rigidbodies!");

	auto rb = rigidbodies + rigidbody_count++;

	rb->sim = this;

	rb->restitution = 1.0f;
	rb->mass = 1.0;
	rb->stat_friction = 0.1f;
	rb->kin_friction = 0.1f;

	rb->color = { 1.0f, 1.0f, 1.0f };

	rb->position = { 0, 0 };
	rb->velocity = { 0, 0 };

	return rb;
}

Rigidbody* RigidbodySim::new_circle(float radius) {
	auto rb = new_rigidbody();

	rb->type = Rigidbody::circle;
	rb->shape.circle.radius = radius;

	return rb;
}

Rigidbody* RigidbodySim::new_aabb(glm::vec2 size) {
	auto rb = new_rigidbody();

	rb->type = Rigidbody::aabb;
	rb->shape.aabb.size = size;

	return rb;
}

Rigidbody* RigidbodySim::new_plane(glm::vec2 normal) {
	auto rb = new_rigidbody();

	rb->type = Rigidbody::plane;
	rb->shape.plane.normal = normal;

	return rb;
}
