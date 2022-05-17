attribute vec3 attr_Position;
attribute vec3 attr_Normal;

attribute vec4 attr_Color;
attribute vec4 attr_TexCoord0;

uniform mat4 u_ModelMatrix;


uniform vec4   u_BaseColor;
uniform vec4   u_VertColor;

// Uniforms
layout(shared) uniform ViewMatrices
{
    uniform mat4 u_ViewMatrices[NUM_VIEWS];
};
layout(shared) uniform ProjectionMatrix
{
    uniform mat4 u_ProjectionMatrix;
};

varying vec2   var_DiffuseTex;
varying vec4   var_Color;

void main()
{
	vec3 position  = attr_Position;
	vec3 normal    = attr_Normal;

	gl_Position = u_ProjectionMatrix * (u_ViewMatrices[gl_ViewID_OVR] * (u_ModelMatrix * vec4(position, 1.0)));

    var_DiffuseTex = attr_TexCoord0.st;
	var_Color = u_VertColor * attr_Color + u_BaseColor;
	var_Color.rgb = var_Color.rgb * 0.1 + abs(normal.xyz) * 0.9;
}
