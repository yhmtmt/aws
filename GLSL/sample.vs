#version 150

uniform mat4 Mmvp;
uniform mat4 Mm;
uniform int mode;

in vec4 position;
in vec3 normal;
in vec2 texcoord;
in vec2 pos2d;

out vec2 Texcd;
out vec3 Normal;
flat out int Mode;

void main()
{
	Texcd = texcoord;
	Mode = mode;
	switch(mode){
	case 0:
    	Normal = (Mm * vec4(normal, 1)).xyz;
        gl_Position = Mmvp * position ;
		break;
	case 1:
	    Normal = vec3(0.0, 0.0, 0.0);
		gl_Position = vec4(pos2d, 0.0, 1.0);
		break;
	case 2:
		gl_Position = Mmvp * position;
		break;
	}
}

