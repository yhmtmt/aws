#version 130

uniform vec3 Lpar;
uniform sampler2D sampler;
uniform vec4 gcolor;
uniform vec4 gcolorb;

in vec2 Texcd;
in vec3 Normal;
flat in int Mode;
out vec4 outputF;
 
void main()
{
    vec4 clr;
    float cosTheta;

    switch(Mode){
    case 0: // 3d obj mode
        clr = texture(sampler, Texcd);
        cosTheta = dot(Lpar, Normal);   
        outputF = min(1.0, cosTheta + 0.1) * clr;
        break;
    case 1: // 2d text mode
        clr = texture(sampler, Texcd);
        outputF = clr[0] * gcolor + (1.0 - clr[0]) * gcolorb;
        break;
    case 2: // 3d line mode
        outputF = gcolor;    
        break;
    case 3: // 2d obj mode
        outputF = gcolor;
        break;
    }
}