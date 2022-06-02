#version 330 core
precision highp float;

uniform vec4 u_color;

void main()
{
    //vec4 col = texture2D(u_texture, v_uv);
    //gl_FragColor = col * u_color;
    gl_FragColor = vec4(1.0f, 0.0f, 0.0f, 1.0f); // this is making the edge color 
}