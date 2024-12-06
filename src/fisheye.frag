#version 330 core
 
in vec3 TexCoords;

layout (location = 0) out vec3 Color;

uniform sampler2D sampler;
uniform float intr[23];
uniform int width;
uniform int height;

vec2 getPixel(vec3 TexCoords)
{
	vec2 pixel;
	float norm=sqrt(TexCoords[0]*TexCoords[0]+TexCoords[1]*TexCoords[1]);
	float theta=atan(TexCoords[2]/norm);
	float t, t_i;
	float rho, x, y;
	if (norm != 0) {
		t = theta;
		rho = intr[5];
		t_i = 1;
		for (int i = 6; i < 23; i++) {
			t_i *= t;
			rho += t_i*intr[i];
		}
		x = -TexCoords[1] * rho / norm;
		y = TexCoords[0] * rho / norm;
		pixel[0] = x*intr[0] + y*intr[1] + intr[3];
		pixel[1] = x*intr[2] + y + intr[4];
	}
	else {
		pixel[0] = intr[3];
		pixel[1] = intr[4];
	}
	return vec2(pixel[1]/width,pixel[0]/height);
}

void main(){
  Color = texture2D(sampler, getPixel(TexCoords)).zyx;
  //Color=vec3(getPixel(TexCoords),0);
}