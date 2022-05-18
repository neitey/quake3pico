uniform sampler2D u_DiffuseMap;

uniform int       u_AlphaTest;

varying vec2      var_DiffuseTex;

varying vec4      var_Color;

varying highp vec4 clipPos;
varying highp vec4 prevClipPos;

void main()
{
	vec4 color  = texture2D(u_DiffuseMap, var_DiffuseTex);

	float alpha = color.a * var_Color.a;
	if (u_AlphaTest == 1)
	{
		if (alpha == 0.0)
			discard;
	}
	else if (u_AlphaTest == 2)
	{
		if (alpha >= 0.5)
			discard;
	}
	else if (u_AlphaTest == 3)
	{
		if (alpha < 0.5)
			discard;
	}

    gl_FragColor = ( clipPos / clipPos.w ) - ( prevClipPos / prevClipPos.w );
	gl_FragColor.a = alpha == 1.0 ? 1.0 : 0.0;
}
