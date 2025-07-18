#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in int hand_type;

flat out int vHandType;

void main() {
    gl_Position = vec4(aPos, 1.0);
    vHandType = hand_type;
}
