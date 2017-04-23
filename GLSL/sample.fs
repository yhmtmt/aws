#version 150
 
uniform vec3 Lpar;
uniform sampler2D sampler;
uniform vec4 gcolor;

in vec2 Texcd;
in vec3 Normal;
flat in int Mode;
out vec4 outputF;
 
void main()
{
    vec4 clr;
    float cosTheta;

    switch(Mode){
    case 0: // normal mode
        clr = texture(sampler, Texcd);
        cosTheta = dot(Lpar, Normal);   
        outputF = min(1.0, cosTheta + 0.1) * clr;
        break;
    case 1: // text mode
        clr = texture(sampler, Texcd);
       if(clr[0] == 0.0)
            outputF = vec4(0.0, 0.0, 0.0, 0.0);
        else
            outputF = clr[0] * gcolor;
        break;
    case 2: // line mode
        outputF = gcolor;    
        break;
    }
}