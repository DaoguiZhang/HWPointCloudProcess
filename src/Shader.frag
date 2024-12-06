#version 430 core

in vec2 TexCoord0;

uniform sampler2D sampler;

void main(){
	gl_FragColor = texture2D(sampler, TexCoord0);
}