#version 330 core

layout (location = 0) in vec3 Position;
layout (location = 1) in vec3 Color;

uniform mat4 mvMatrix;
uniform mat4 projMatrix;

out vec3 ourcolor;

void main(){
	gl_Position = projMatrix * mvMatrix * vec4(Position, 1.0f);
	ourcolor=Color;
}