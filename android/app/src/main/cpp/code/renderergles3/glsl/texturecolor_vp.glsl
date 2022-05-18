attribute vec3 attr_Position;
attribute vec4 attr_TexCoord0;

// Uniforms

uniform mat4 u_ModelMatrix;

layout(shared) uniform ViewProjectionMatrices
{
    uniform mat4 u_ViewProjectionMatrices[NUM_VIEWS];
};

varying vec2   var_Tex1;


void main()
{
	gl_Position = u_ViewProjectionMatrices[gl_ViewID_OVR] * (u_ModelMatrix * vec4(attr_Position, 1.0));

	var_Tex1 = attr_TexCoord0.st;
}
