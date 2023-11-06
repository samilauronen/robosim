#version 330
layout(location = 0) in vec4 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aColor;

out vec3 FragPos;
out vec3 Normal;
out vec4 vColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
	FragPos = vec3(model * aPosition);
	Normal = mat3(transpose(inverse(model))) * aNormal;
	vColor = aColor;

	gl_Position = projection * view * vec4(FragPos, 1.0);
}