#version 410
layout (location = 0) in vec3 vertexPosition;
uniform vec4 color;
uniform mat4 camera;
uniform mat4 world;
uniform mat4 proj;
out vec4 ourColor;

void main()
{
    //gl_PointSize = 1.0f;
    ourColor = color;
    gl_Position = proj * camera * world * vec4(vertexPosition.x,vertexPosition.y,-vertexPosition.z,1.0);
}

