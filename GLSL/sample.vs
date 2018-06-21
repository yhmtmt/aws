#version 130

 
uniform vec2 inv_sz_half_scrn; 
uniform mat4 Mmvp;
uniform mat4 Mm;
uniform int mode;
uniform float depth2d;

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
	case 0: // 3d obj mode
    	Normal = (Mm * vec4(normal, 1)).xyz;
        gl_Position = Mmvp * position ;
		break;
	case 1: // 2d text mode
	    Normal = vec3(0.0, 0.0, 0.0);
		gl_Position = vec4(pos2d.x * inv_sz_half_scrn.x, pos2d.y * inv_sz_half_scrn.y, depth2d, 1.0);
		break;
	case 2: // 3d line mode
		gl_Position = Mmvp * position;
		break;
	case 3:// 2d obj mode
	    Normal = vec3(0.0, 0.0, 0.0);
		gl_Position = vec4(pos2d.x * inv_sz_half_scrn.x, pos2d.y * inv_sz_half_scrn.y, depth2d, 1.0);
		break;
	}
}

