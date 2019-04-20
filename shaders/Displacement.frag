#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_2, uv).r;
}

void main() {
  // YOUR CODE HERE

  vec3 v_pos = v_position.xyz;

  float w = u_texture_2_size.x;
  float height = u_texture_2_size.y;

  vec3 n = normalize(v_normal.xyz);
  vec3 t = normalize(v_tangent.xyz);
  vec3 b = normalize(cross(n, t));

  mat3 TBN = mat3(t, b, n);

  vec2 new_uv_U = vec2(v_uv.x + 1 /w, v_uv.y);
  vec2 new_uv_V = vec2(v_uv.x, v_uv.y + 1 / height);

  float dU = (h(new_uv_U) - h(v_uv)) * u_normal_scaling * u_height_scaling;
  float dV = (h(new_uv_V) - h(v_uv)) * u_normal_scaling * u_height_scaling;

  vec3 new_normal = TBN * vec3(-dU, -dV, 1);

  float ka = 0.25;
  float kd = 1.0;
  float ks = 1.0;
  vec3 ia = vec3(1.0, 1.0, 1.0);
  float p = 100.0;

  vec3 v_norm = normalize(new_normal);

  float r = length(u_light_pos - v_pos);
  vec3 intensity = u_light_intensity / (r * r);
  vec3 l = u_light_pos - v_pos;
  vec3 v = u_cam_pos - v_pos;
  vec3 h = (v + l) / length(v + l);

  // (Placeholder code. You will want to replace it.)
  out_color = vec4(ka * ia + kd * intensity * max(0.0, dot(normalize(l), v_norm)) +
  ks * intensity * pow(max(0.0, dot(v_norm, h)), p), 1);
  out_color.a = 1;
}