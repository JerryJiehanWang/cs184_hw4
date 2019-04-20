#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  float ka = 0.25;
  float kd = 1.0;
  float ks = 1.0;
  vec3 ia = vec3(0.5, 0.5, 0.5);
  float p = 100.0;

  vec3 v_pos = v_position.xyz;
  vec3 v_norm = normalize(v_normal.xyz);

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

