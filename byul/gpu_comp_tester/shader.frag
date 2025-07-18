#version 330 core
out vec4 FragColor;

flat in int vHandType;

void main() {
    if (vHandType == 0)
        FragColor = vec4(1, 0, 0, 1); // SLERP → 빨강
    else
        FragColor = vec4(0, 0.5, 1, 1); // LERP → 파랑
}
