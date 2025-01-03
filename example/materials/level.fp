varying mediump vec2 var_texcoord0;
uniform lowp sampler2D tex0;

varying highp vec4 var_position;
uniform mediump vec4 entity;

void main()
{
    gl_FragColor = texture2D(tex0, var_texcoord0.xy);
    float d = length(var_position - entity);
    if (d < 2.) {
        gl_FragColor = mix(vec4(0.), gl_FragColor, d * 0.5);
    }
}
