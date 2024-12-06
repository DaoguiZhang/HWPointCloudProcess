#version 430 core

layout (location = 0) in vec3 Position;
layout (location = 1) in vec2 TexCoord;

uniform mat4 mvMatrix;
uniform mat4 projMatrix;

out vec2 TexCoord0;

void main(){
	gl_Position = projMatrix * mvMatrix * vec4(Position, 1.0f);
	TexCoord0 = TexCoord;
}