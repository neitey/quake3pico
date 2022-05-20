attribute vec3 attr_Position;
attribute vec3 attr_Normal;

attribute vec4 attr_Color;
attribute vec4 attr_TexCoord0;

uniform mat4 u_ModelMatrix;
uniform mat4 u_PrevModelMatrix;


uniform vec4   u_BaseColor;
uniform vec4   u_VertColor;

// Uniforms
layout(shared) uniform ViewProjectionMatrices
{
    uniform mat4 u_ViewProjectionMatrices[NUM_VIEWS];
};
layout(shared) uniform PrevViewProjectionMatrices
{
    uniform mat4 u_PrevViewProjectionMatrices[NUM_VIEWS];
};

varying vec2   var_DiffuseTex;
varying vec4   var_Color;

varying highp vec4 clipPos;
varying highp vec4 prevClipPos;

void main()
{
	vec3 position  = attr_Position;
	vec3 normal    = attr_Normal;

	clipPos = u_ViewProjectionMatrices[gl_ViewID_OVR] * (u_ModelMatrix * vec4(position, 1.0));
	prevClipPos = u_PrevViewProjectionMatrices[gl_ViewID_OVR] * (u_PrevModelMatrix * vec4(position, 1.0));

    var_DiffuseTex = attr_TexCoord0.st;
	var_Color = u_VertColor * attr_Color + u_BaseColor;
	var_Color.rgb = var_Color.rgb * 0.1 + abs(normal.xyz) * 0.9;
	
	gl_Position = clipPos;
}
