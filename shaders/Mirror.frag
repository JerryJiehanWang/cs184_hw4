#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec4 wo = vec4(u_cam_pos, 1) - v_position;
  vec4 wi = (-wo - 2 * dot(-wo, v_normal) * v_normal);
  out_color = texture(u_texture_cubemap, vec3(wi.xyz[0], wi.xyz[1], wi.xyz[2]));
  out_color.a = 1;
}
