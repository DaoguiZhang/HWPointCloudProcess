#version 330 core
 
in vec3 TexCoords;
//in vec3 LocalPos;

layout (location = 0) out vec3 Color;
//layout (location = 1) out uint Depth;

uniform sampler2D sampler;
 
vec2 coordsInEqurectMap(vec3 TexCoords)
{
    const float pi = 3.14159265359;
    const float pi_mult_2 = pi*2;

    vec3 normCubeCoords = normalize(TexCoords);
    float lon = atan(normCubeCoords.z, normCubeCoords.x) + pi;
    float lat = acos(normCubeCoords.y);

    float lon_norm = lon/pi_mult_2;
    float lat_norm = lat/pi;

    return vec2(lon_norm, lat_norm);
}
 
void main(){
  //gl_FragColor = texture2D(sampler, coordsInEqurectMap(TexCoords));
  Color = texture2D(sampler, coordsInEqurectMap(TexCoords)).zyx;
  //Depth = uint(LocalPos.z * 1000);
}