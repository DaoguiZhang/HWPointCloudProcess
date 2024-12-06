#version 330 core
in vec3 ourcolor;

void main(){
	gl_FragColor = vec4(ourcolor,1.0f);
}