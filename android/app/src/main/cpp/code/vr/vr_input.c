#include "vr_input.h"

//#if __ANDROID__

#include "../qcommon/qcommon.h"
#include "../client/keycodes.h"
#include "../client/client.h"
#include "vr_base.h"
#include "vr_clientinfo.h"

#include <unistd.h>
#include <jni.h>

#ifdef USE_LOCAL_HEADERS
#	include "SDL.h"
#else
#	include <SDL.h>
#endif

XrResult CheckXrResult(XrResult res, const char* originator) {
    if (XR_FAILED(res)) {
        Com_Printf("error: %s", originator);
    }
    return res;
}

#define CHECK_XRCMD(cmd) CheckXrResult(cmd, #cmd);

#define SIDE_LEFT 0
#define SIDE_RIGHT 1
#define SIDE_COUNT 2


XrActionSet actionSet;
XrAction grabAction;
XrAction poseAction;
XrAction vibrateAction;
XrAction quitAction;
/*************************pico******************/
XrAction touchpadAction;
XrAction AXAction;
XrAction homeAction;
XrAction BYAction;
XrAction backAction;
XrAction sideAction;
XrAction triggerAction;
XrAction joystickAction;
XrAction batteryAction;
//---add new----------
XrAction AXTouchAction;
XrAction BYTouchAction;
XrAction RockerTouchAction;
XrAction TriggerTouchAction;
XrAction ThumbrestTouchAction;
XrAction GripAction;
//---add new----------zgt
XrAction AAction;
XrAction BAction;
XrAction XAction;
XrAction YAction;
XrAction ATouchAction;
XrAction BTouchAction;
XrAction XTouchAction;
XrAction YTouchAction;
XrAction aimAction;
/*************************pico******************/
XrSpace aimSpace[SIDE_COUNT];
XrPath handSubactionPath[SIDE_COUNT];
XrSpace handSpace[SIDE_COUNT];

qboolean inputInitialized = qfalse;

void ResetInput()
{
    actionSet = XR_NULL_HANDLE;
    grabAction = XR_NULL_HANDLE;
    poseAction = XR_NULL_HANDLE;
    vibrateAction = XR_NULL_HANDLE;
    quitAction = XR_NULL_HANDLE;
    /*************************pico******************/
    touchpadAction = XR_NULL_HANDLE;
    AXAction = XR_NULL_HANDLE;
    homeAction = XR_NULL_HANDLE;
    BYAction = XR_NULL_HANDLE;
    backAction = XR_NULL_HANDLE;
    sideAction = XR_NULL_HANDLE;
    triggerAction = XR_NULL_HANDLE;
    joystickAction = XR_NULL_HANDLE;
    batteryAction = XR_NULL_HANDLE;
    //---add new----------
    AXTouchAction = XR_NULL_HANDLE;
    BYTouchAction = XR_NULL_HANDLE;
    RockerTouchAction = XR_NULL_HANDLE;
    TriggerTouchAction = XR_NULL_HANDLE;
    ThumbrestTouchAction = XR_NULL_HANDLE;
    GripAction = XR_NULL_HANDLE;
    //---add new----------zgt
    AAction = XR_NULL_HANDLE;
    BAction = XR_NULL_HANDLE;
    XAction = XR_NULL_HANDLE;
    YAction = XR_NULL_HANDLE;
    ATouchAction = XR_NULL_HANDLE;
    BTouchAction = XR_NULL_HANDLE;
    XTouchAction = XR_NULL_HANDLE;
    YTouchAction = XR_NULL_HANDLE;
    aimAction = XR_NULL_HANDLE;
}

enum {
	VR_TOUCH_AXIS_UP = 1 << 0,
	VR_TOUCH_AXIS_UPRIGHT = 1 << 1,
	VR_TOUCH_AXIS_RIGHT = 1 << 2,
	VR_TOUCH_AXIS_DOWNRIGHT = 1 << 3,
	VR_TOUCH_AXIS_DOWN = 1 << 4,
	VR_TOUCH_AXIS_DOWNLEFT = 1 << 5,
	VR_TOUCH_AXIS_LEFT = 1 << 6,
	VR_TOUCH_AXIS_UPLEFT = 1 << 7,
	VR_TOUCH_AXIS_TRIGGER_INDEX = 1 << 8,
};

typedef struct {
	uint32_t buttons;
	uint32_t axisButtons;
} vrController_t;

vr_clientinfo_t vr;

static qboolean controllerInit = qfalse;

static vrController_t leftController;
static vrController_t rightController;
static int in_vrEventTime = 0;
static double lastframetime = 0;

static float triggerPressedThreshold = 0.75f;
static float triggerReleasedThreshold = 0.5f;

static float thumbstickPressedThreshold = 0.5f;
static float thumbstickReleasedThreshold = 0.4f;

extern cvar_t *cl_sensitivity;
extern cvar_t *m_pitch;
extern cvar_t *m_yaw;

float radians(float deg) {
    return (deg * M_PI) / 180.0;
}

float degrees(float rad) {
    return (rad * 180.0) / M_PI;
}


#ifndef EPSILON
#define EPSILON 0.001f
#endif

extern cvar_t *vr_righthanded;
extern cvar_t *vr_switchThumbsticks;
extern cvar_t *vr_snapturn;
extern cvar_t *vr_extralatencymode;
extern cvar_t *vr_directionMode;
extern cvar_t *vr_weaponPitch;
extern cvar_t *vr_heightAdjust;
extern cvar_t *vr_twoHandedWeapons;
extern cvar_t *vr_refreshrate;
extern cvar_t *vr_weaponScope;
extern cvar_t *vr_hapticIntensity;

jclass callbackClass;
jmethodID android_haptic_event;

qboolean alt_key_mode_active = qfalse;

void rotateAboutOrigin(float x, float y, float rotation, vec2_t out)
{
	out[0] = cosf(DEG2RAD(-rotation)) * x  +  sinf(DEG2RAD(-rotation)) * y;
	out[1] = cosf(DEG2RAD(-rotation)) * y  -  sinf(DEG2RAD(-rotation)) * x;
}

XrVector3f normalizeVec(XrVector3f vec) {
    //NOTE: leave w-component untouched
    //@@const float EPSILON = 0.000001f;
    float xxyyzz = vec.x*vec.x + vec.y*vec.y + vec.z*vec.z;
    //@@if(xxyyzz < EPSILON)
    //@@    return *this; // do nothing if it is zero vector

    //float invLength = invSqrt(xxyyzz);
    XrVector3f result;
    float invLength = 1.0f / sqrtf(xxyyzz);
    result.x = vec.x * invLength;
    result.y = vec.y * invLength;
    result.z = vec.z * invLength;
    return result;
}

static float length(float x, float y)
{
    return sqrtf(powf(x, 2.0f) + powf(y, 2.0f));
}

void NormalizeAngles(vec3_t angles)
{
    while (angles[0] >= 90) angles[0] -= 180;
    while (angles[1] >= 180) angles[1] -= 360;
    while (angles[2] >= 180) angles[2] -= 360;
    while (angles[0] < -90) angles[0] += 180;
    while (angles[1] < -180) angles[1] += 360;
    while (angles[2] < -180) angles[2] += 360;
}

void GetAnglesFromVectors(const XrVector3f forward, const XrVector3f right, const XrVector3f up, vec3_t angles)
{
    float sr, sp, sy, cr, cp, cy;

    sp = -forward.z;

    float cp_x_cy = forward.x;
    float cp_x_sy = forward.y;
    float cp_x_sr = -right.z;
    float cp_x_cr = up.z;

    float yaw = atan2(cp_x_sy, cp_x_cy);
    float roll = atan2(cp_x_sr, cp_x_cr);

    cy = cos(yaw);
    sy = sin(yaw);
    cr = cos(roll);
    sr = sin(roll);

    if (fabs(cy) > EPSILON)
    {
        cp = cp_x_cy / cy;
    }
    else if (fabs(sy) > EPSILON)
    {
        cp = cp_x_sy / sy;
    }
    else if (fabs(sr) > EPSILON)
    {
        cp = cp_x_sr / sr;
    }
    else if (fabs(cr) > EPSILON)
    {
        cp = cp_x_cr / cr;
    }
    else
    {
        cp = cos(asin(sp));
    }

    float pitch = atan2(sp, cp);

    angles[0] = pitch / (M_PI*2.f / 360.f);
    angles[1] = yaw / (M_PI*2.f / 360.f);
    angles[2] = roll / (M_PI*2.f / 360.f);

    NormalizeAngles(angles);
}

void QuatToYawPitchRoll(XrQuaternionf q, vec3_t rotation, vec3_t out) {

    ovrMatrix4f mat = ovrMatrix4f_CreateFromQuaternion( &q );

    if (rotation[0] != 0.0f || rotation[1] != 0.0f || rotation[2] != 0.0f)
    {
        ovrMatrix4f rot = ovrMatrix4f_CreateRotation(radians(rotation[0]), radians(rotation[1]), radians(rotation[2]));
        mat = ovrMatrix4f_Multiply(&mat, &rot);
    }

    XrVector4f v1 = {0, 0, -1, 0};
    XrVector4f v2 = {1, 0, 0, 0};
    XrVector4f v3 = {0, 1, 0, 0};

    XrVector4f forwardInVRSpace = XrVector4f_MultiplyMatrix4f(&mat, &v1);
    XrVector4f rightInVRSpace = XrVector4f_MultiplyMatrix4f(&mat, &v2);
    XrVector4f upInVRSpace = XrVector4f_MultiplyMatrix4f(&mat, &v3);

    XrVector3f forward = {-forwardInVRSpace.z, -forwardInVRSpace.x, forwardInVRSpace.y};
    XrVector3f right = {-rightInVRSpace.z, -rightInVRSpace.x, rightInVRSpace.y};
    XrVector3f up = {-upInVRSpace.z, -upInVRSpace.x, upInVRSpace.y};

    XrVector3f forwardNormal = normalizeVec(forward);
    XrVector3f rightNormal = normalizeVec(right);
    XrVector3f upNormal = normalizeVec(up);

    GetAnglesFromVectors(forwardNormal, rightNormal, upNormal, out);
}

//0 = left, 1 = right
float vibration_channel_duration[2] = {0.0f, 0.0f};
float vibration_channel_intensity[2] = {0.0f, 0.0f};

void VR_Vibrate( int duration, int chan, float intensity )
{
    for (int i = 0; i < 2; ++i)
    {
        int channel = (i + 1) & chan;
        if (channel)
        {
            if (vibration_channel_duration[channel-1] > 0.0f)
                return;

            if (vibration_channel_duration[channel-1] == -1.0f && duration != 0.0f)
                return;

            vibration_channel_duration[channel-1] = duration;
            vibration_channel_intensity[channel-1] = intensity * vr_hapticIntensity->value;
        }
    }
}


static void VR_processHaptics() {
    static float lastFrameTime = 0.0f;
    float timestamp = (float)(Sys_Milliseconds( ));
    float frametime = timestamp - lastFrameTime;
    lastFrameTime = timestamp;

    for (int i = 0; i < 2; ++i) {
        if (vibration_channel_duration[i] > 0.0f ||
            vibration_channel_duration[i] == -1.0f) {

            // fire haptics using output action
            XrHapticVibration vibration = {};
            vibration.type = XR_TYPE_HAPTIC_VIBRATION;
            vibration.next = NULL;
            vibration.amplitude = vibration_channel_intensity[i];
            vibration.duration = ToXrTime(vibration_channel_duration[i]);
            vibration.frequency = 3000;
            XrHapticActionInfo hapticActionInfo = {};
            hapticActionInfo.type = XR_TYPE_HAPTIC_ACTION_INFO;
            hapticActionInfo.next = NULL;
            hapticActionInfo.action = vibrateAction;
            hapticActionInfo.subactionPath = handSubactionPath[i];
            OXR(xrApplyHapticFeedback(VR_GetEngine()->appState.Session, &hapticActionInfo, (const XrHapticBaseHeader*)&vibration));

            if (vibration_channel_duration[i] != -1.0f) {
                vibration_channel_duration[i] -= frametime;

                if (vibration_channel_duration[i] < 0.0f) {
                    vibration_channel_duration[i] = 0.0f;
                    vibration_channel_intensity[i] = 0.0f;
                }
            }
        } else {
            // Stop haptics
            XrHapticActionInfo hapticActionInfo = {};
            hapticActionInfo.type = XR_TYPE_HAPTIC_ACTION_INFO;
            hapticActionInfo.next = NULL;
            hapticActionInfo.action = vibrateAction;
            hapticActionInfo.subactionPath = handSubactionPath[i];
            OXR(xrStopHapticFeedback(VR_GetEngine()->appState.Session, &hapticActionInfo));
        }
    }
}

static qboolean IN_GetInputAction(const char* inputName, char* action)
{
    char cvarname[256];
    Com_sprintf(cvarname, 256, "vr_button_map_%s%s", inputName, alt_key_mode_active ? "_ALT" : "");
    char * val = Cvar_VariableString(cvarname);
    if (val && strlen(val) > 0)
    {
        Com_sprintf(action, 256, "%s", val);
        return qtrue;
    }

    //If we didn't find something for this input and the alt key is active, then see if the un-alt key has a function
    if (alt_key_mode_active)
    {
        Com_sprintf(cvarname, 256, "vr_button_map_%s", inputName);
        char * val = Cvar_VariableString(cvarname);
        if (val && strlen(val) > 0)
        {
            Com_sprintf(action, 256, "%s", val);
            return qtrue;
        }
    }

    return qfalse;
}

// Returns true in case active input should be auto-repeated (now only applicable for smooth-turn)
static qboolean IN_SendInputAction(const char* action, qboolean inputActive, float axisValue, qboolean thumbstickAxis)
{
    if (action)
    {
        //handle our special actions first
        if (strcmp(action, "blank") == 0) {
            // Empty function used to block alt fallback on unmapped alt buttons or
            // force 8-way mapping mode of thumbstick without assigning actual action
        }
        else if (strcmp(action, "+alt") == 0)
        {
            alt_key_mode_active = inputActive;
        }
        else if (strcmp(action, "+weapon_stabilise") == 0)
        {
            //stabilised weapon only triggered when controllers close enough (40cm) to each other
            if (inputActive)
            {
                vec3_t l;
                VectorSubtract(vr.weaponposition, vr.offhandposition, l);
                vr.weapon_stabilised =  VectorLength(l) < 0.4f;
            }
            else
            {
                vr.weapon_stabilised =  qfalse;
            }
        }
        else if (strcmp(action, "+weapon_select") == 0)
        {
            vr.weapon_select = inputActive;
            if (inputActive) {
                int selectorType = (int) Cvar_VariableValue("vr_weaponSelectorMode");
                vr.weapon_select_using_thumbstick = (selectorType == WS_HMD);
                vr.weapon_select_autoclose = vr.weapon_select_using_thumbstick && thumbstickAxis;
            } else {
                vr.weapon_select_using_thumbstick = qfalse;
                vr.weapon_select_autoclose = qfalse;
                Cbuf_AddText("weapon_select");
            }
        }
        else if (action[0] == '+')
        {
            char command[256];
            Com_sprintf(command, sizeof(command), "%s%s\n", inputActive ? "+" : "-", action + 1);
            Cbuf_AddText(command);
        }
        else if (inputActive)
        {
            if (strcmp(action, "turnleft") == 0) {
                if (vr_snapturn->integer > 0) { // snap turn
                    int snap = 45;
                    if (vr_snapturn->integer > 1) {
                        snap = vr_snapturn->integer;
                    }
                    CL_SnapTurn(-snap);
                } else { // yaw (smooth turn)
                    float value = (axisValue > 0.0f ? axisValue : 1.0f) * cl_sensitivity->value * m_yaw->value;
                    Com_QueueEvent(in_vrEventTime, SE_MOUSE, -value, 0, 0, NULL);
                    return qtrue;
                }
            } else if (strcmp(action, "turnright") == 0) {
                if (vr_snapturn->integer > 0) { // snap turn
                    int snap = 45;
                    if (vr_snapturn->integer > 1) {
                        snap = vr_snapturn->integer;
                    }
                    CL_SnapTurn(snap);
                } else { // yaw (smooth turn)
                    float value = (axisValue > 0.0f ? axisValue : 1.0f) * cl_sensitivity->value * m_yaw->value;
                    Com_QueueEvent(in_vrEventTime, SE_MOUSE, value, 0, 0, NULL);
                    return qtrue;
                }
            } else if (strcmp(action, "uturn") == 0) {
                CL_SnapTurn(180);
            } else {
                char command[256];
                Com_sprintf(command, sizeof(command), "%s\n", action);
                Cbuf_AddText(command);
            }
        }
    }
    return qfalse;
}

static void IN_ActivateInput(uint32_t * inputGroup, int inputFlag) {
    *inputGroup |= inputFlag;
}

static void IN_DeactivateInput(uint32_t * inputGroup, int inputFlag) {
    *inputGroup &= ~inputFlag;
}

static qboolean IN_InputActivated(uint32_t * inputGroup, int inputFlag) {
    return (*inputGroup & inputFlag);
}

static void IN_HandleActiveInput(uint32_t * inputGroup, int inputFlag, char* inputName, float axisValue, qboolean thumbstickAxis) {
    if (IN_InputActivated(inputGroup, inputFlag)) {
        // Input is already in activated state, nothing to do
        return;
    }
    char action[256];
    if (IN_GetInputAction(inputName, action)) {
        // Activate input action
        if (!IN_SendInputAction(action, qtrue, axisValue, thumbstickAxis)) {
            // Action should not be repeated, mark input as activated
            IN_ActivateInput(inputGroup, inputFlag);
        }
    } else {
        // No assigned action -> mark input as activated
        // (to avoid unnecessary action lookup next time)
        IN_ActivateInput(inputGroup, inputFlag);
    }
}

static void IN_HandleInactiveInput(uint32_t * inputGroup, int inputFlag, char* inputName, float axisValue, qboolean thumbstickAxis) {
    if (!IN_InputActivated(inputGroup, inputFlag)) {
        // Input is not in activated state, nothing to do
        return;
    }
    char action[256];
    if (IN_GetInputAction(inputName, action)) {
        // Deactivate input action and remove input activated state
        IN_SendInputAction(action, qfalse, axisValue, thumbstickAxis);
        IN_DeactivateInput(inputGroup, inputFlag);
    } else {
        // No assigned action -> just remove input activated state
        IN_DeactivateInput(inputGroup, inputFlag);
    }
}

void VR_HapticEvent(const char* event, int position, int flags, int intensity, float angle, float yHeight )
{
    /*if (vr_hapticIntensity->value == 0.0f)
    {
        return;
    }

    engine_t* engine = VR_GetEngine();
    jstring StringArg1 = (*(engine->java.Env))->NewStringUTF(engine->java.Env, event);
    (*(engine->java.Env))->CallVoidMethod(engine->java.Env, engine->java.ActivityObject, android_haptic_event, StringArg1, position, flags, (int)(intensity * vr_hapticIntensity->value), angle, yHeight);

    //Controller Haptic Support
    int weaponFireChannel = vr.weapon_stabilised ? 3 : (vr_righthanded->integer ? 2 : 1);
    if (strcmp(event, "pickup_shield") == 0 ||
            strcmp(event, "pickup_weapon") == 0 ||
            strstr(event, "pickup_item") != NULL)
    {
        VR_Vibrate(100, 3, 1.0);
    }
    else if (strcmp(event, "weapon_switch") == 0)
    {
        VR_Vibrate(250, vr_righthanded->integer ? 2 : 1, 0.8);
    }
    else if (strcmp(event, "shotgun") == 0 || strcmp(event, "fireball") == 0)
    {
        VR_Vibrate(400, 3, 1.0);
    }
    else if (strcmp(event, "bullet") == 0)
    {
        VR_Vibrate(150, 3, 1.0);
    }
    else if (strcmp(event, "chainsaw_fire") == 0 ||
        strcmp(event, "RTCWQuest:fire_tesla") == 0)
    {
        VR_Vibrate(500, weaponFireChannel, 1.0);
    }
    else if (strcmp(event, "machinegun_fire") == 0 || strcmp(event, "plasmagun_fire") == 0)
    {
        VR_Vibrate(90, weaponFireChannel, 0.8);
    }
    else if (strcmp(event, "shotgun_fire") == 0)
    {
        VR_Vibrate(250, weaponFireChannel, 1.0);
    }
    else if (strcmp(event, "rocket_fire") == 0 ||
        strcmp(event, "RTCWQuest:fire_sniper") == 0 ||
        strcmp(event, "bfg_fire") == 0 ||
        strcmp(event, "handgrenade_fire") == 0 )
    {
        VR_Vibrate(400, weaponFireChannel, 1.0);
    }
    else if (strcmp(event, "selector_icon") == 0)
    {
        //Quick blip
        VR_Vibrate(50, (vr_righthanded->integer ? 2 : 1), 1.0);
    }*/
}

XrActionSuggestedBinding ActionSuggestedBinding(XrAction action, XrPath path) {
    XrActionSuggestedBinding asb;
    asb.action = action;
    asb.binding = path;
    return asb;
}

XrActionStateBoolean GetActionStateBoolean(XrAction action, int hand) {
    XrActionStateGetInfo getInfo = {};
    getInfo.type = XR_TYPE_ACTION_STATE_GET_INFO;
    getInfo.action = action;
    if (hand >= 0)
        getInfo.subactionPath = handSubactionPath[hand];

    XrActionStateBoolean state = {};
    state.type = XR_TYPE_ACTION_STATE_BOOLEAN;
    CHECK_XRCMD(xrGetActionStateBoolean(VR_GetEngine()->appState.Session, &getInfo, &state));
    return state;
}

XrActionStateFloat GetActionStateFloat(XrAction action, int hand) {
    XrActionStateGetInfo getInfo = {};
    getInfo.type = XR_TYPE_ACTION_STATE_GET_INFO;
    getInfo.action = action;
    if (hand >= 0)
        getInfo.subactionPath = handSubactionPath[hand];

    XrActionStateFloat state = {};
    state.type = XR_TYPE_ACTION_STATE_FLOAT;
    CHECK_XRCMD(xrGetActionStateFloat(VR_GetEngine()->appState.Session, &getInfo, &state));
    return state;
}

XrActionStateVector2f GetActionStateVector2(XrAction action, int hand) {
    XrActionStateGetInfo getInfo = {};
    getInfo.type = XR_TYPE_ACTION_STATE_GET_INFO;
    getInfo.action = action;
    if (hand >= 0)
        getInfo.subactionPath = handSubactionPath[hand];

    XrActionStateVector2f state = {};
    state.type = XR_TYPE_ACTION_STATE_VECTOR2F;
    CHECK_XRCMD(xrGetActionStateVector2f(VR_GetEngine()->appState.Session, &getInfo, &state));
    return state;
}

void InitializeActions() {
    engine_t* engine = VR_GetEngine();

    // Create an action set.
    {
        XrActionSetCreateInfo actionSetInfo = {};
        actionSetInfo.type = XR_TYPE_ACTION_SET_CREATE_INFO;
        strcpy(actionSetInfo.actionSetName, "gameplay");
        strcpy(actionSetInfo.localizedActionSetName, "Gameplay");
        actionSetInfo.priority = 0;
        CHECK_XRCMD(xrCreateActionSet(engine->appState.Instance, &actionSetInfo, &actionSet));
    }

    // Get the XrPath for the left and right hands - we will use them as subaction paths.
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left", &handSubactionPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right", &handSubactionPath[SIDE_RIGHT]));

    // Create actions.
    {
        // Create an input action for grabbing objects with the left and right hands.
        XrActionCreateInfo actionInfo = {};
        actionInfo.type = XR_TYPE_ACTION_CREATE_INFO;
        actionInfo.actionType = XR_ACTION_TYPE_FLOAT_INPUT;
        strcpy(actionInfo.actionName, "grab_object");
        strcpy(actionInfo.localizedActionName, "Grab Object");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &grabAction));

        // Create an input action getting the left and right hand poses.
        actionInfo.actionType = XR_ACTION_TYPE_POSE_INPUT;
        strcpy(actionInfo.actionName, "hand_pose");
        strcpy(actionInfo.localizedActionName, "Hand Pose");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &poseAction));

        actionInfo.actionType = XR_ACTION_TYPE_POSE_INPUT;
        strcpy(actionInfo.actionName, "aim_pose");
        strcpy(actionInfo.localizedActionName, "Aim Pose");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &aimAction));

        // Create output actions for vibrating the left and right controller.
        actionInfo.actionType = XR_ACTION_TYPE_VIBRATION_OUTPUT;
        strcpy(actionInfo.actionName, "vibrate_hand");
        strcpy(actionInfo.localizedActionName, "Vibrate Hand");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &vibrateAction));

        // Create input actions for quitting the session using the left and right controller.
        // Since it doesn't matter which hand did this, we do not specify subaction paths for it.
        // We will just suggest bindings for both hands, where possible.
        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "quit_session");
        strcpy(actionInfo.localizedActionName, "Quit Session");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &quitAction));
        /**********************************pico***************************************/
        // Create input actions for toucpad key using the left and right controller.
        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "touchpad");
        strcpy(actionInfo.localizedActionName, "Touchpad");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &touchpadAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "axkey");
        strcpy(actionInfo.localizedActionName, "AXkey");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &AXAction));


        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "homekey");
        strcpy(actionInfo.localizedActionName, "Homekey");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &homeAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "bykey");
        strcpy(actionInfo.localizedActionName, "BYkey");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &BYAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "backkey");
        strcpy(actionInfo.localizedActionName, "Backkey");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &backAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "sidekey");
        strcpy(actionInfo.localizedActionName, "Sidekey");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &sideAction));

        actionInfo.actionType = XR_ACTION_TYPE_FLOAT_INPUT;
        strcpy(actionInfo.actionName, "trigger");
        strcpy(actionInfo.localizedActionName, "Trigger");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &triggerAction));

        actionInfo.actionType = XR_ACTION_TYPE_VECTOR2F_INPUT;
        strcpy(actionInfo.actionName, "joystick");
        strcpy(actionInfo.localizedActionName, "Joystick");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &joystickAction));

        actionInfo.actionType = XR_ACTION_TYPE_FLOAT_INPUT;
        strcpy(actionInfo.actionName, "battery");
        strcpy(actionInfo.localizedActionName, "battery");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &batteryAction));
        //------------------------add new---------------------------------
        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "axtouch");
        strcpy(actionInfo.localizedActionName, "AXtouch");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &AXTouchAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "bytouch");
        strcpy(actionInfo.localizedActionName, "BYtouch");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &BYTouchAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "rockertouch");
        strcpy(actionInfo.localizedActionName, "Rockertouch");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &RockerTouchAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "triggertouch");
        strcpy(actionInfo.localizedActionName, "Triggertouch");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &TriggerTouchAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "thumbresttouch");
        strcpy(actionInfo.localizedActionName, "Thumbresttouch");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &ThumbrestTouchAction));

        actionInfo.actionType = XR_ACTION_TYPE_FLOAT_INPUT;
        strcpy(actionInfo.actionName, "gripvalue");
        strcpy(actionInfo.localizedActionName, "GripValue");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &GripAction));

        //--------------add new----------zgt
        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "akey");
        strcpy(actionInfo.localizedActionName, "Akey");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &AAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "bkey");
        strcpy(actionInfo.localizedActionName, "Bkey");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &BAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "xkey");
        strcpy(actionInfo.localizedActionName, "Xkey");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &XAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "ykey");
        strcpy(actionInfo.localizedActionName, "Ykey");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &YAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "atouch");
        strcpy(actionInfo.localizedActionName, "Atouch");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &ATouchAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "btouch");
        strcpy(actionInfo.localizedActionName, "Btouch");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &BTouchAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "xtouch");
        strcpy(actionInfo.localizedActionName, "Xtouch");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &XTouchAction));

        actionInfo.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
        strcpy(actionInfo.actionName, "ytouch");
        strcpy(actionInfo.localizedActionName, "Ytouch");
        actionInfo.countSubactionPaths = SIDE_COUNT;
        actionInfo.subactionPaths = handSubactionPath;
        CHECK_XRCMD(xrCreateAction(actionSet, &actionInfo, &YTouchAction));
        /**********************************pico***************************************/


    }

    XrPath selectPath[SIDE_COUNT];
    XrPath squeezeValuePath[SIDE_COUNT];
    XrPath squeezeClickPath[SIDE_COUNT];
    XrPath posePath[SIDE_COUNT];
    XrPath hapticPath[SIDE_COUNT];
    XrPath menuClickPath[SIDE_COUNT];
    XrPath systemPath[SIDE_COUNT];
    XrPath thumbrestPath[SIDE_COUNT];
    XrPath triggerTouchPath[SIDE_COUNT];
    XrPath triggerValuePath[SIDE_COUNT];
    XrPath thumbstickClickPath[SIDE_COUNT];
    XrPath thumbstickTouchPath[SIDE_COUNT];
    XrPath thumbstickPosPath[SIDE_COUNT];
    XrPath aimPath[SIDE_COUNT];

    /**************************pico************************************/
    XrPath touchpadPath[SIDE_COUNT];
    XrPath AXValuePath[SIDE_COUNT];
    XrPath homeClickPath[SIDE_COUNT];
    XrPath BYValuePath[SIDE_COUNT];
    XrPath backPath[SIDE_COUNT];
    XrPath sideClickPath[SIDE_COUNT];
    XrPath triggerPath[SIDE_COUNT];
    XrPath joystickPath[SIDE_COUNT];
    XrPath batteryPath[SIDE_COUNT];
    //--------------add new----------
    XrPath GripPath[SIDE_COUNT];
    XrPath AXTouchPath[SIDE_COUNT];
    XrPath BYTouchPath[SIDE_COUNT];
    XrPath RockerTouchPath[SIDE_COUNT];
    XrPath TriggerTouchPath[SIDE_COUNT];
    XrPath ThumbresetTouchPath[SIDE_COUNT];
    //--------------add new----------zgt
    XrPath AValuePath[SIDE_COUNT];
    XrPath BValuePath[SIDE_COUNT];
    XrPath XValuePath[SIDE_COUNT];
    XrPath YValuePath[SIDE_COUNT];
    XrPath ATouchPath[SIDE_COUNT];
    XrPath BTouchPath[SIDE_COUNT];
    XrPath XTouchPath[SIDE_COUNT];
    XrPath YTouchPath[SIDE_COUNT];
    /**************************pico************************************/
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/select/click", &selectPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/select/click", &selectPath[SIDE_RIGHT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/menu/click", &menuClickPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/menu/click", &menuClickPath[SIDE_RIGHT]));

    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/squeeze/value", &squeezeValuePath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/squeeze/value", &squeezeValuePath[SIDE_RIGHT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/squeeze/click", &squeezeClickPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/squeeze/click", &squeezeClickPath[SIDE_RIGHT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/grip/pose", &posePath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/grip/pose", &posePath[SIDE_RIGHT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/aim/pose", &aimPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/aim/pose", &aimPath[SIDE_RIGHT]));

    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/output/haptic", &hapticPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/output/haptic", &hapticPath[SIDE_RIGHT]));

    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/trigger/touch", &triggerTouchPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/trigger/touch", &triggerTouchPath[SIDE_RIGHT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/trigger/value", &triggerValuePath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/trigger/value", &triggerValuePath[SIDE_RIGHT]));

    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/thumbstick/click", &thumbstickClickPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/thumbstick/click", &thumbstickClickPath[SIDE_RIGHT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/thumbstick/touch", &thumbstickTouchPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/thumbstick/touch", &thumbstickTouchPath[SIDE_RIGHT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/thumbstick", &thumbstickPosPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/thumbstick", &thumbstickPosPath[SIDE_RIGHT]));

    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/system/click", &systemPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/system/click", &systemPath[SIDE_RIGHT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/thumbrest/touch", &thumbrestPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/thumbrest/touch", &thumbrestPath[SIDE_RIGHT]));

    /**************************pico************************************/
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/back/click", &backPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/back/click", &backPath[SIDE_RIGHT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/battery/value", &batteryPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/battery/value", &batteryPath[SIDE_RIGHT]));

    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/x/click", &XValuePath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/y/click", &YValuePath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/a/click", &AValuePath[SIDE_RIGHT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/b/click", &BValuePath[SIDE_RIGHT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/x/touch", &XTouchPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/left/input/y/touch", &YTouchPath[SIDE_LEFT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/a/touch", &ATouchPath[SIDE_RIGHT]));
    CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/user/hand/right/input/b/touch", &BTouchPath[SIDE_RIGHT]));
    /**************************pico************************************/
    XrActionSuggestedBinding bindings[128];
    int currBinding = 0;

    // Suggest bindings for KHR Simple.
    /* {
        XrPath khrSimpleInteractionProfilePath;
         CHECK_XRCMD(
             xrStringToPath(up.mInstance, "/interaction_profiles/khr/simple_controller", &khrSimpleInteractionProfilePath));
         bindings[currBinding++] = ActionSuggestedBinding(grabAction, selectPath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(grabAction, selectPath[SIDE_RIGHT]);
         bindings[currBinding++] = ActionSuggestedBinding(poseAction, posePath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(poseAction, posePath[SIDE_RIGHT]);
         bindings[currBinding++] = ActionSuggestedBinding(quitAction, menuClickPath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(quitAction, menuClickPath[SIDE_RIGHT]);
         bindings[currBinding++] = ActionSuggestedBinding(vibrateAction, hapticPath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(vibrateAction, hapticPath[SIDE_RIGHT]);
         XrInteractionProfileSuggestedBinding suggestedBindings = {};
         suggestedBindings.type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING;
         suggestedBindings.interactionProfile = khrSimpleInteractionProfilePath;
         suggestedBindings.suggestedBindings = bindings;
         suggestedBindings.countSuggestedBindings = currBinding;
         CHECK_XRCMD(xrSuggestInteractionProfileBindings(engine->appState.Instance, &suggestedBindings));
     }*/
     // Suggest bindings for the Oculus Touch.
     /*{
         XrPath oculusTouchInteractionProfilePath;
         CHECK_XRCMD(
             xrStringToPath(engine->appState.Instance, "/interaction_profiles/oculus/touch_controller", &oculusTouchInteractionProfilePath));
         bindings[currBinding++] = ActionSuggestedBinding(grabAction, triggerValuePath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(grabAction, triggerValuePath[SIDE_RIGHT]);
         bindings[currBinding++] = ActionSuggestedBinding(poseAction, posePath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(poseAction, posePath[SIDE_RIGHT]);
         bindings[currBinding++] = ActionSuggestedBinding(aimAction, aimPath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(aimAction, aimPath[SIDE_RIGHT]);
         bindings[currBinding++] = ActionSuggestedBinding(quitAction, menuClickPath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(vibrateAction, hapticPath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(vibrateAction, hapticPath[SIDE_RIGHT]);
         XrInteractionProfileSuggestedBinding suggestedBindings = {};
         suggestedBindings.type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING;
         suggestedBindings.interactionProfile = oculusTouchInteractionProfilePath;
         suggestedBindings.suggestedBindings = bindings;
         suggestedBindings.countSuggestedBindings = currBinding;
         CHECK_XRCMD(xrSuggestInteractionProfileBindings(engine->appState.Instance, &suggestedBindings));
     }*/
     // Suggest bindings for the Vive Controller.
     /*{
         XrPath viveControllerInteractionProfilePath;
         CHECK_XRCMD(
             xrStringToPath(engine->appState.Instance, "/interaction_profiles/htc/vive_controller", &viveControllerInteractionProfilePath));
         bindings[currBinding++] = ActionSuggestedBinding(grabAction, squeezeClickPath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(grabAction, squeezeClickPath[SIDE_RIGHT]);
         bindings[currBinding++] = ActionSuggestedBinding(poseAction, posePath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(poseAction, posePath[SIDE_RIGHT]);
         bindings[currBinding++] = ActionSuggestedBinding(quitAction, menuClickPath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(quitAction, menuClickPath[SIDE_RIGHT]);
         bindings[currBinding++] = ActionSuggestedBinding(vibrateAction, hapticPath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(vibrateAction, hapticPath[SIDE_RIGHT]);
         XrInteractionProfileSuggestedBinding suggestedBindings = {};
         suggestedBindings.type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING;
         suggestedBindings.interactionProfile = viveControllerInteractionProfilePath;
         suggestedBindings.suggestedBindings = bindings;
         suggestedBindings.countSuggestedBindings = currBinding;
         CHECK_XRCMD(xrSuggestInteractionProfileBindings(engine->appState.Instance, &suggestedBindings));
     }*/

     // Suggest bindings for the Microsoft Mixed Reality Motion Controller.
     /*{
         XrPath microsoftMixedRealityInteractionProfilePath;
         CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/interaction_profiles/microsoft/motion_controller",
                                    &microsoftMixedRealityInteractionProfilePath));
         bindings[currBinding++] = ActionSuggestedBinding(grabAction, squeezeClickPath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(grabAction, squeezeClickPath[SIDE_RIGHT]);
         bindings[currBinding++] = ActionSuggestedBinding(poseAction, posePath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(poseAction, posePath[SIDE_RIGHT]);
         bindings[currBinding++] = ActionSuggestedBinding(quitAction, menuClickPath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(quitAction, menuClickPath[SIDE_RIGHT]);
         bindings[currBinding++] = ActionSuggestedBinding(vibrateAction, hapticPath[SIDE_LEFT]);
         bindings[currBinding++] = ActionSuggestedBinding(vibrateAction, hapticPath[SIDE_RIGHT]);
         XrInteractionProfileSuggestedBinding suggestedBindings = {};
         suggestedBindings.type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING;
         suggestedBindings.interactionProfile = microsoftMixedRealityInteractionProfilePath;
         suggestedBindings.suggestedBindings = bindings.data();
         suggestedBindings.countSuggestedBindings = (uint32_t)bindings.size();
         CHECK_XRCMD(xrSuggestInteractionProfileBindings(engine->appState.Instance, &suggestedBindings));
     }*/
    // Suggest bindings for the Pico Neo 3 controller
    {
        XrPath picoMixedRealityInteractionProfilePath;
        CHECK_XRCMD(xrStringToPath(engine->appState.Instance, "/interaction_profiles/pico/neo3_controller",
                                   &picoMixedRealityInteractionProfilePath));

        bindings[currBinding++] = ActionSuggestedBinding(touchpadAction, thumbstickClickPath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(touchpadAction, thumbstickClickPath[SIDE_RIGHT]);
        bindings[currBinding++] = ActionSuggestedBinding(joystickAction, thumbstickPosPath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(joystickAction, thumbstickPosPath[SIDE_RIGHT]);
        bindings[currBinding++] = ActionSuggestedBinding(RockerTouchAction, thumbstickTouchPath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(RockerTouchAction, thumbstickTouchPath[SIDE_RIGHT]);

        bindings[currBinding++] = ActionSuggestedBinding(triggerAction, triggerValuePath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(triggerAction, triggerValuePath[SIDE_RIGHT]);
        bindings[currBinding++] = ActionSuggestedBinding(TriggerTouchAction, triggerTouchPath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(TriggerTouchAction, triggerTouchPath[SIDE_RIGHT]);

        bindings[currBinding++] = ActionSuggestedBinding(sideAction, squeezeClickPath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(sideAction, squeezeClickPath[SIDE_RIGHT]);
        bindings[currBinding++] = ActionSuggestedBinding(GripAction, squeezeValuePath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(GripAction, squeezeValuePath[SIDE_RIGHT]);
        bindings[currBinding++] = ActionSuggestedBinding(poseAction, posePath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(poseAction, posePath[SIDE_RIGHT]);

        bindings[currBinding++] = ActionSuggestedBinding(homeAction, systemPath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(homeAction, systemPath[SIDE_RIGHT]);
        bindings[currBinding++] = ActionSuggestedBinding(backAction, backPath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(backAction, backPath[SIDE_RIGHT]);
        bindings[currBinding++] = ActionSuggestedBinding(batteryAction, batteryPath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(batteryAction, batteryPath[SIDE_RIGHT]);

        bindings[currBinding++] = ActionSuggestedBinding(ThumbrestTouchAction, thumbrestPath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(ThumbrestTouchAction, thumbrestPath[SIDE_RIGHT]);

        bindings[currBinding++] = ActionSuggestedBinding(XTouchAction, XTouchPath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(YTouchAction, YTouchPath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(ATouchAction, ATouchPath[SIDE_RIGHT]);
        bindings[currBinding++] = ActionSuggestedBinding(BTouchAction, BTouchPath[SIDE_RIGHT]);
        bindings[currBinding++] = ActionSuggestedBinding(XAction, XValuePath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(YAction, YValuePath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(AAction, AValuePath[SIDE_RIGHT]);
        bindings[currBinding++] = ActionSuggestedBinding(BAction, BValuePath[SIDE_RIGHT]);
        bindings[currBinding++] = ActionSuggestedBinding(aimAction, aimPath[SIDE_LEFT]);
        bindings[currBinding++] = ActionSuggestedBinding(aimAction, aimPath[SIDE_RIGHT]);

        XrInteractionProfileSuggestedBinding suggestedBindings = {};
        suggestedBindings.type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING;
        suggestedBindings.interactionProfile = picoMixedRealityInteractionProfilePath;
        suggestedBindings.suggestedBindings = bindings;
        suggestedBindings.countSuggestedBindings = currBinding;
        CHECK_XRCMD(xrSuggestInteractionProfileBindings(engine->appState.Instance, &suggestedBindings));
    }

    XrActionSpaceCreateInfo actionSpaceInfo = {};
    actionSpaceInfo.type = XR_TYPE_ACTION_SPACE_CREATE_INFO;
    actionSpaceInfo.action = poseAction;
    actionSpaceInfo.poseInActionSpace.orientation.w = 1.f;
    actionSpaceInfo.subactionPath = handSubactionPath[SIDE_LEFT];
    CHECK_XRCMD(xrCreateActionSpace(engine->appState.Session, &actionSpaceInfo, &handSpace[SIDE_LEFT]));
    actionSpaceInfo.subactionPath = handSubactionPath[SIDE_RIGHT];
    CHECK_XRCMD(xrCreateActionSpace(engine->appState.Session, &actionSpaceInfo, &handSpace[SIDE_RIGHT]));
    actionSpaceInfo.action = aimAction;
    actionSpaceInfo.poseInActionSpace.orientation.w = 1.f;
    actionSpaceInfo.subactionPath = handSubactionPath[SIDE_LEFT];
    CHECK_XRCMD(xrCreateActionSpace(engine->appState.Session, &actionSpaceInfo, &aimSpace[SIDE_LEFT]));
    actionSpaceInfo.subactionPath = handSubactionPath[SIDE_RIGHT];
    CHECK_XRCMD(xrCreateActionSpace(engine->appState.Session, &actionSpaceInfo, &aimSpace[SIDE_RIGHT]));

    XrSessionActionSetsAttachInfo attachInfo = {};
    attachInfo.type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO;
    attachInfo.countActionSets = 1;
    attachInfo.actionSets = &actionSet;
    CHECK_XRCMD(xrAttachSessionActionSets(engine->appState.Session, &attachInfo));
}

void IN_VRInit( void )
{
    ResetInput();
    inputInitialized = qfalse;
}

static void IN_VRController( qboolean isRightController, XrPosef pose )
{
    //Set gun angles - We need to calculate all those we might need (including adjustments) for the client to then take its pick
    vec3_t rotation = {0};
    if (isRightController == (vr_righthanded->integer != 0))
    {
        //Set gun angles - We need to calculate all those we might need (including adjustments) for the client to then take its pick
        rotation[PITCH] = vr_weaponPitch->value;
        QuatToYawPitchRoll(pose.orientation, rotation, vr.weaponangles);

        VectorSubtract(vr.weaponangles_last, vr.weaponangles, vr.weaponangles_delta);
        VectorCopy(vr.weaponangles, vr.weaponangles_last);

        ///Weapon location relative to view
        vr.weaponposition[0] = pose.position.x;
        vr.weaponposition[1] = pose.position.y + vr_heightAdjust->value;
        vr.weaponposition[2] = pose.position.z;

        VectorCopy(vr.weaponoffset_last[1], vr.weaponoffset_last[0]);
        VectorCopy(vr.weaponoffset, vr.weaponoffset_last[1]);
        VectorSubtract(vr.weaponposition, vr.hmdposition, vr.weaponoffset);
    } else {
        QuatToYawPitchRoll(pose.orientation, rotation, vr.offhandangles2); // used for off-hand direction mode
        rotation[PITCH] = vr_weaponPitch->value;
        QuatToYawPitchRoll(pose.orientation, rotation, vr.offhandangles);

        ///location relative to view
        vr.offhandposition[0] = pose.position.x;
        vr.offhandposition[1] = pose.position.y + vr_heightAdjust->value;
        vr.offhandposition[2] = pose.position.z;

        VectorCopy(vr.offhandoffset_last[1], vr.offhandoffset_last[0]);
        VectorCopy(vr.offhandoffset, vr.offhandoffset_last[1]);
        VectorSubtract(vr.offhandposition, vr.hmdposition, vr.offhandoffset);
    }

	if (vr.virtual_screen || cl.snap.ps.pm_type == PM_INTERMISSION)
    {
        vr.weapon_zoomed = qfalse;
        if (vr.menuCursorX && vr.menuCursorY)
        {
            float yaw;
            float pitch;
            if (vr.menuLeftHanded) {
                yaw = (vr_righthanded->integer != 0) ? vr.offhandangles[YAW] : vr.weaponangles[YAW];
                pitch = (vr_righthanded->integer != 0) ? vr.offhandangles[PITCH] : vr.weaponangles[PITCH];
            } else {
                yaw = (vr_righthanded->integer != 0) ? vr.weaponangles[YAW] : vr.offhandangles[YAW];
                pitch = (vr_righthanded->integer != 0) ? vr.weaponangles[PITCH] : vr.offhandangles[PITCH];
            }
            int x = 320 - tan((yaw - vr.menuYaw) * (M_PI*2 / 360)) * 400;
            int y = 240 + tan((pitch + vr_weaponPitch->value) * (M_PI*2 / 360)) * 400;
            *vr.menuCursorX = x;
            *vr.menuCursorY = y;
            Com_QueueEvent(in_vrEventTime, SE_MOUSE, 0, 0, 0, NULL);
        }
    }
    else
    {

        vr.weapon_zoomed = vr_weaponScope->integer &&
                           vr.weapon_stabilised &&
                           (cl.snap.ps.weapon == WP_RAILGUN) &&
                           (VectorLength(vr.weaponoffset) < 0.24f) &&
                           cl.snap.ps.stats[STAT_HEALTH] > 0;

        if (vr_twoHandedWeapons->integer && vr.weapon_stabilised)
        {
            //Apply smoothing to the weapon hand
            vec3_t smooth_weaponoffset;
            VectorAdd(vr.weaponoffset, vr.weaponoffset_last[0], smooth_weaponoffset);
            VectorAdd(smooth_weaponoffset, vr.weaponoffset_last[1],smooth_weaponoffset);
            VectorScale(smooth_weaponoffset, 1.0f/3.0f, smooth_weaponoffset);

            vec3_t vec;
            VectorSubtract(vr.offhandoffset, smooth_weaponoffset, vec);

            float zxDist = length(vec[0], vec[2]);

            if (zxDist != 0.0f && vec[2] != 0.0f) {
                VectorSet(vr.weaponangles, -degrees(atanf(vec[1] / zxDist)),
                          -degrees(atan2f(vec[0], -vec[2])), vr.weaponangles[ROLL] / 2.0f); //Dampen roll on stabilised weapon
            }
        }
    }
}

static qboolean IN_VRJoystickUse8WayMapping( void ) {
    char action[256];
    return IN_GetInputAction("RTHUMBFORWARDRIGHT", action)
        || IN_GetInputAction("RTHUMBBACKRIGHT", action)
        || IN_GetInputAction("RTHUMBBACKLEFT", action)
        || IN_GetInputAction("RTHUMBFORWARDLEFT", action);
}

static void IN_VRJoystickHandle4WayMapping( uint32_t * inputGroup, float joystickAngle, float joystickValue ) {
    if (joystickAngle >= 315.0 || joystickAngle < 45.0) { // UP
        // Deactivate neighboring inputs
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UPRIGHT, "RTHUMBFORWARDRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_RIGHT, "RTHUMBRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_LEFT, "RTHUMBLEFT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UPLEFT, "RTHUMBFORWARDLEFT", joystickValue, qtrue);
        // Activate UP
        IN_HandleActiveInput(inputGroup, VR_TOUCH_AXIS_UP, "RTHUMBFORWARD", joystickValue, qtrue);
    } else if (joystickAngle < 135.0) { // RIGHT
        // Deactivate neighboring inputs
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UP, "RTHUMBFORWARD", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UPRIGHT, "RTHUMBFORWARDRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWNRIGHT, "RTHUMBBACKRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWN, "RTHUMBBACK", joystickValue, qtrue);
        // Activate RIGHT
        IN_HandleActiveInput(inputGroup, VR_TOUCH_AXIS_RIGHT, "RTHUMBRIGHT", joystickValue, qtrue);
    } else if (joystickAngle < 225.0) { // DOWN
        // Deactivate neighboring inputs
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_RIGHT, "RTHUMBRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWNRIGHT, "RTHUMBBACKRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWNLEFT, "RTHUMBBACKLEFT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_LEFT, "RTHUMBLEFT", joystickValue, qtrue);
        // Activate DOWN
        IN_HandleActiveInput(inputGroup, VR_TOUCH_AXIS_DOWN, "RTHUMBBACK", joystickValue, qtrue);
    } else { // LEFT
        // Deactivate neighboring inputs
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWN, "RTHUMBBACK", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWNLEFT, "RTHUMBBACKLEFT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UPLEFT, "RTHUMBFORWARDLEFT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UP, "RTHUMBFORWARD", joystickValue, qtrue);
        // Activate LEFT
        IN_HandleActiveInput(inputGroup, VR_TOUCH_AXIS_LEFT, "RTHUMBLEFT", joystickValue, qtrue);
    }
}

static void IN_VRJoystickHandle8WayMapping( uint32_t * inputGroup, float joystickAngle, float joystickValue ) {
    if (joystickAngle > 337.5 || joystickAngle < 22.5) { // UP
        // Deactivate neighboring inputs
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UPRIGHT, "RTHUMBFORWARDRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_RIGHT, "RTHUMBRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_LEFT, "RTHUMBLEFT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UPLEFT, "RTHUMBFORWARDLEFT", joystickValue, qtrue);
        // Activate UP
        IN_HandleActiveInput(inputGroup, VR_TOUCH_AXIS_UP, "RTHUMBFORWARD", joystickValue, qtrue);
    } else if (joystickAngle < 67.5) { // UP-RIGHT
        // Deactivate neighboring inputs
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UP, "RTHUMBFORWARD", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_RIGHT, "RTHUMBRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWNRIGHT, "RTHUMBBACKRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UPLEFT, "RTHUMBFORWARDLEFT", joystickValue, qtrue);
        // Activate UP-RIGHT
        IN_HandleActiveInput(inputGroup, VR_TOUCH_AXIS_UPRIGHT, "RTHUMBFORWARDRIGHT", joystickValue, qtrue);
    } else if (joystickAngle < 112.5) { // RIGHT
        // Deactivate neighboring inputs
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UP, "RTHUMBFORWARD", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UPRIGHT, "RTHUMBFORWARDRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWNRIGHT, "RTHUMBBACKRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWN, "RTHUMBBACK", joystickValue, qtrue);
        // Activate RIGHT
        IN_HandleActiveInput(inputGroup, VR_TOUCH_AXIS_RIGHT, "RTHUMBRIGHT", joystickValue, qtrue);
    } else if (joystickAngle < 157.5) { // DOWN-RIGHT
        // Deactivate neighboring inputs
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UPRIGHT, "RTHUMBFORWARDRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_RIGHT, "RTHUMBRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWN, "RTHUMBBACK", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWNLEFT, "RTHUMBBACKLEFT", joystickValue, qtrue);
        // Activate DOWN-RIGHT
        IN_HandleActiveInput(inputGroup, VR_TOUCH_AXIS_DOWNRIGHT, "RTHUMBBACKRIGHT", joystickValue, qtrue);
    } else if (joystickAngle < 202.5) { // DOWN
        // Deactivate neighboring inputs
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_RIGHT, "RTHUMBRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWNRIGHT, "RTHUMBBACKRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWNLEFT, "RTHUMBBACKLEFT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_LEFT, "RTHUMBLEFT", joystickValue, qtrue);
        // Activate DOWN
        IN_HandleActiveInput(inputGroup, VR_TOUCH_AXIS_DOWN, "RTHUMBBACK", joystickValue, qtrue);
    } else if (joystickAngle < 247.5) { // DOWN-LEFT
        // Deactivate neighboring inputs
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWNRIGHT, "RTHUMBBACKRIGHT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWN, "RTHUMBBACK", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_LEFT, "RTHUMBLEFT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UPLEFT, "RTHUMBFORWARDLEFT", joystickValue, qtrue);
        // Activate DOWN-LEFT
        IN_HandleActiveInput(inputGroup, VR_TOUCH_AXIS_DOWNLEFT, "RTHUMBBACKLEFT", joystickValue, qtrue);
    } else if (joystickAngle < 292.5) { // LEFT
        // Deactivate neighboring inputs
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWN, "RTHUMBBACK", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWNLEFT, "RTHUMBBACKLEFT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UPLEFT, "RTHUMBFORWARDLEFT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UP, "RTHUMBFORWARD", joystickValue, qtrue);
        // Activate LEFT
        IN_HandleActiveInput(inputGroup, VR_TOUCH_AXIS_LEFT, "RTHUMBLEFT", joystickValue, qtrue);
    } else { // UP-LEFT
        // Deactivate neighboring inputs
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_DOWNLEFT, "RTHUMBBACKLEFT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_LEFT, "RTHUMBLEFT", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UP, "RTHUMBFORWARD", joystickValue, qtrue);
        IN_HandleInactiveInput(inputGroup, VR_TOUCH_AXIS_UPRIGHT, "RTHUMBFORWARDRIGHT", joystickValue, qtrue);
        // Activate UP-LEFT
        IN_HandleActiveInput(inputGroup, VR_TOUCH_AXIS_UPLEFT, "RTHUMBFORWARDLEFT", joystickValue, qtrue);
    }
}

static void IN_VRJoystick( qboolean isRightController, float joystickX, float joystickY )
{
	vrController_t* controller = isRightController == qtrue ? &rightController : &leftController;

    vr.thumbstick_location[isRightController][0] = joystickX;
    vr.thumbstick_location[isRightController][1] = joystickY;

	if (vr.virtual_screen || cl.snap.ps.pm_type == PM_INTERMISSION)
	{

	    // Use thumbstick UP/DOWN as PAGEUP/PAGEDOWN in menus
        if (joystickY > thumbstickPressedThreshold) {
            if (!IN_InputActivated(&controller->axisButtons, VR_TOUCH_AXIS_UP)) {
                IN_ActivateInput(&controller->axisButtons, VR_TOUCH_AXIS_UP);
                Com_QueueEvent(in_vrEventTime, SE_KEY, K_PGUP, qtrue, 0, NULL);
            }
        } else if (joystickY < -thumbstickPressedThreshold) {
            if (!IN_InputActivated(&controller->axisButtons, VR_TOUCH_AXIS_DOWN)) {
                IN_ActivateInput(&controller->axisButtons, VR_TOUCH_AXIS_DOWN);
                Com_QueueEvent(in_vrEventTime, SE_KEY, K_PGDN, qtrue, 0, NULL);
            }
        } else if (joystickY < thumbstickReleasedThreshold && joystickY > -thumbstickReleasedThreshold) {
            if (IN_InputActivated(&controller->axisButtons, VR_TOUCH_AXIS_UP)) {
                IN_DeactivateInput(&controller->axisButtons, VR_TOUCH_AXIS_UP);
                Com_QueueEvent(in_vrEventTime, SE_KEY, K_PGUP, qfalse, 0, NULL);
            }
            if (IN_InputActivated(&controller->axisButtons, VR_TOUCH_AXIS_DOWN)) {
                IN_DeactivateInput(&controller->axisButtons, VR_TOUCH_AXIS_DOWN);
                Com_QueueEvent(in_vrEventTime, SE_KEY, K_PGDN, qfalse, 0, NULL);
            }
        }

	}
	else
	{

		if (isRightController == (vr_switchThumbsticks->integer != 0)) {
			vec3_t positional;
			VectorClear(positional);

			vec2_t joystick;
            if ( vr.use_fake_6dof )
            {
                //multiplayer game
                if (!vr_directionMode->integer) {
					//HMD Based
					rotateAboutOrigin(joystickX, joystickY, vr.hmdorientation[YAW], joystick);
				} else {
                	//Off-hand based
					rotateAboutOrigin(joystickX, joystickY, vr.offhandangles2[YAW], joystick);
				}
            }
            else
            {
				//Positional movement speed correction for when we are not hitting target framerate
				float refresh = 1000.0f / (in_vrEventTime - lastframetime);
				//TODO:VR_GetEngine()->appState.pfnGetDisplayRefreshRate(VR_GetEngine()->appState.Session, &refresh);
				float factor = (refresh / 72.0F) * 10.0f; // adjust positional factor based on refresh rate
				rotateAboutOrigin(-vr.hmdposition_delta[0] * factor,
								  vr.hmdposition_delta[2] * factor, -vr.hmdorientation[YAW], positional);

				if (!vr_directionMode->integer) {
					//HMD Based
					joystick[0] = joystickX;
					joystick[1] = joystickY;
				} else {
					//Off-hand based
					rotateAboutOrigin(joystickX, joystickY, vr.offhandangles2[YAW] - vr.hmdorientation[YAW], joystick);
				}
            }

            //sideways
            Com_QueueEvent(in_vrEventTime, SE_JOYSTICK_AXIS, 0, joystick[0] * 127.0f + positional[0] * 127.0f, 0, NULL);

            //forward/back
            Com_QueueEvent(in_vrEventTime, SE_JOYSTICK_AXIS, 1, joystick[1] * 127.0f + positional[1] * 127.0f, 0, NULL);
        }

        // In case thumbstick is used by weapon wheel (is in HMD/thumbstick mode), ignore standard thumbstick inputs
        else if (!vr.weapon_select_using_thumbstick)
        {
            float joystickValue = length(joystickX, joystickY);
            if (joystickValue < thumbstickReleasedThreshold) {
                // Joystick within threshold -> disable all inputs
                IN_HandleInactiveInput(&controller->axisButtons, VR_TOUCH_AXIS_UP, "RTHUMBFORWARD", joystickValue, qtrue);
                IN_HandleInactiveInput(&controller->axisButtons, VR_TOUCH_AXIS_UPRIGHT, "RTHUMBFORWARDRIGHT", joystickValue, qtrue);
                IN_HandleInactiveInput(&controller->axisButtons, VR_TOUCH_AXIS_RIGHT, "RTHUMBRIGHT", joystickValue, qtrue);
                IN_HandleInactiveInput(&controller->axisButtons, VR_TOUCH_AXIS_DOWNRIGHT, "RTHUMBBACKRIGHT", joystickValue, qtrue);
                IN_HandleInactiveInput(&controller->axisButtons, VR_TOUCH_AXIS_DOWN, "RTHUMBBACK", joystickValue, qtrue);
                IN_HandleInactiveInput(&controller->axisButtons, VR_TOUCH_AXIS_DOWNLEFT, "RTHUMBBACKLEFT", joystickValue, qtrue);
                IN_HandleInactiveInput(&controller->axisButtons, VR_TOUCH_AXIS_LEFT, "RTHUMBLEFT", joystickValue, qtrue);
                IN_HandleInactiveInput(&controller->axisButtons, VR_TOUCH_AXIS_UPLEFT, "RTHUMBFORWARDLEFT", joystickValue, qtrue);
            } else if (joystickValue > thumbstickPressedThreshold) {
                float joystickAngle = AngleNormalize360(RAD2DEG(atan2(joystickX, joystickY)));
                if (IN_VRJoystickUse8WayMapping()) {
                    IN_VRJoystickHandle8WayMapping(&controller->axisButtons, joystickAngle, joystickValue);
                } else {
                    IN_VRJoystickHandle4WayMapping(&controller->axisButtons, joystickAngle, joystickValue);
                }
            }
        }

    }
}

static void IN_VRTriggers( qboolean isRightController, float triggerValue ) {
	vrController_t* controller = isRightController == qtrue ? &rightController : &leftController;

	if (VR_useScreenLayer() || cl.snap.ps.pm_type == PM_INTERMISSION) {

    	// Triggers are used for menu navigation in screen mode or in intermission
        if (triggerValue > triggerPressedThreshold && !IN_InputActivated(&controller->axisButtons, VR_TOUCH_AXIS_TRIGGER_INDEX))
        {
            IN_ActivateInput(&controller->axisButtons, VR_TOUCH_AXIS_TRIGGER_INDEX);
            if ((isRightController && !vr.menuLeftHanded) || (!isRightController && vr.menuLeftHanded)) {
                // Active controller confirms selection
                Com_QueueEvent(in_vrEventTime, SE_KEY, K_MOUSE1, qtrue, 0, NULL);
                VR_Vibrate(200, vr.menuLeftHanded ? 1 : 2, 0.8);
            } else {
                // Inactive controller becomes active one
                vr.menuLeftHanded = !vr.menuLeftHanded;
            }
        }
        else if (triggerValue < triggerReleasedThreshold && IN_InputActivated(&controller->axisButtons, VR_TOUCH_AXIS_TRIGGER_INDEX))
        {
            IN_DeactivateInput(&controller->axisButtons, VR_TOUCH_AXIS_TRIGGER_INDEX);
            if ((isRightController && !vr.menuLeftHanded) || (!isRightController && vr.menuLeftHanded)) {
                Com_QueueEvent(in_vrEventTime, SE_KEY, K_MOUSE1, qfalse, 0, NULL);
            }
        }

    } else {
        
        // Primary trigger
        if (isRightController == (vr_righthanded->integer != 0))
        {
            if (triggerValue > triggerPressedThreshold) {
                IN_HandleActiveInput(&controller->axisButtons, VR_TOUCH_AXIS_TRIGGER_INDEX, "PRIMARYTRIGGER", triggerValue, qfalse);
            } else if (triggerValue < triggerReleasedThreshold) {
                IN_HandleInactiveInput(&controller->axisButtons, VR_TOUCH_AXIS_TRIGGER_INDEX, "PRIMARYTRIGGER", triggerValue, qfalse);
            }
        }

        // Off hand trigger
        if (isRightController != (vr_righthanded->integer != 0))
        {
            if (triggerValue > triggerPressedThreshold) {
                IN_HandleActiveInput(&controller->axisButtons, VR_TOUCH_AXIS_TRIGGER_INDEX, "SECONDARYTRIGGER", triggerValue, qfalse);
            } else if (triggerValue < triggerReleasedThreshold) {
                IN_HandleInactiveInput(&controller->axisButtons, VR_TOUCH_AXIS_TRIGGER_INDEX, "SECONDARYTRIGGER", triggerValue, qfalse);
            }
        }

    }
}

static void IN_VRButtons( qboolean isRightController, uint32_t buttons )
{
	vrController_t* controller = isRightController == qtrue ? &rightController : &leftController;

    // Menu button
    if ((buttons & ovrButton_Enter) && !IN_InputActivated(&controller->buttons, ovrButton_Enter)) {
        IN_ActivateInput(&controller->buttons, ovrButton_Enter);
        Com_QueueEvent(in_vrEventTime, SE_KEY, K_ESCAPE, qtrue, 0, NULL);
    } else if (!(buttons & ovrButton_Enter) && IN_InputActivated(&controller->buttons, ovrButton_Enter)) {
        IN_DeactivateInput(&controller->buttons, ovrButton_Enter);
        Com_QueueEvent(in_vrEventTime, SE_KEY, K_ESCAPE, qfalse, 0, NULL);
    }

	if (isRightController == !vr_righthanded->integer)
    {
        if (buttons & ovrButton_GripTrigger) {
            IN_HandleActiveInput(&controller->buttons, ovrButton_GripTrigger, "SECONDARYGRIP", 0, qfalse);
        } else {
            IN_HandleInactiveInput(&controller->buttons, ovrButton_GripTrigger, "SECONDARYGRIP", 0, qfalse);
        }
	}
    else
    {
        if (buttons & ovrButton_GripTrigger) {
            IN_HandleActiveInput(&controller->buttons, ovrButton_GripTrigger, "PRIMARYGRIP", 0, qfalse);
        } else {
            IN_HandleInactiveInput(&controller->buttons, ovrButton_GripTrigger, "PRIMARYGRIP", 0, qfalse);
        }
    }

    if (isRightController == !vr_righthanded->integer)
    {
        if (buttons & ovrButton_LThumb) {
            if (!IN_InputActivated(&controller->buttons, ovrButton_LThumb)) {
                // Initiate position reset for fake 6DoF
                vr.realign = 3;
            }
            IN_HandleActiveInput(&controller->buttons, ovrButton_LThumb, "SECONDARYTHUMBSTICK", 0, qfalse);
        } else {
            IN_HandleInactiveInput(&controller->buttons, ovrButton_LThumb, "SECONDARYTHUMBSTICK", 0, qfalse);
        }
    }
    else
    {
        if (buttons & ovrButton_RThumb) {
            IN_HandleActiveInput(&controller->buttons, ovrButton_RThumb, "PRIMARYTHUMBSTICK", 0, qfalse);
        } else {
            IN_HandleInactiveInput(&controller->buttons, ovrButton_RThumb, "PRIMARYTHUMBSTICK", 0, qfalse);
        }
    }

    if (buttons & ovrButton_A) {
        if (cl.snap.ps.pm_flags & PMF_FOLLOW)
        {
            // Go back to free spectator mode if following player
            if (!IN_InputActivated(&controller->buttons, ovrButton_A)) {
                IN_ActivateInput(&controller->buttons, ovrButton_A);
                Cbuf_AddText("cmd team spectator\n");
            }
        }
        else if (VR_useScreenLayer() || cl.snap.ps.pm_type == PM_INTERMISSION)
        {
            // Skip server search in the server menu
            if (!IN_InputActivated(&controller->buttons, ovrButton_A)) {
                IN_ActivateInput(&controller->buttons, ovrButton_A);
                Com_QueueEvent(in_vrEventTime, SE_KEY, K_SPACE, qtrue, 0, NULL);
            }
        }
        else
        {
            IN_HandleActiveInput(&controller->buttons, ovrButton_A, "A", 0, qfalse);
        }
    } else {
        if (VR_useScreenLayer() || cl.snap.ps.pm_type == PM_INTERMISSION)
        {
            // Skip server search in the server menu
            if (IN_InputActivated(&controller->buttons, ovrButton_A)) {
                IN_DeactivateInput(&controller->buttons, ovrButton_A);
                Com_QueueEvent(in_vrEventTime, SE_KEY, K_SPACE, qfalse, 0, NULL);
            }
        }
        else
        {
            IN_HandleInactiveInput(&controller->buttons, ovrButton_A, "A", 0, qfalse);
        }
    }

    if (buttons & ovrButton_B) {
        IN_HandleActiveInput(&controller->buttons, ovrButton_B, "B", 0, qfalse);
    } else {
        IN_HandleInactiveInput(&controller->buttons, ovrButton_B, "B", 0, qfalse);
    }

    if (buttons & ovrButton_X) {
        if (cl.snap.ps.pm_flags & PMF_FOLLOW)
        {
            // Switch follow mode
            if (!IN_InputActivated(&controller->buttons, ovrButton_X)) {
                IN_ActivateInput(&controller->buttons, ovrButton_X);
                vr.follow_mode = (vr.follow_mode+1) % VRFM_NUM_FOLLOWMODES;
            }
        }
        else
        {
            IN_HandleActiveInput(&controller->buttons, ovrButton_X, "X", 0, qfalse);
        }
    } else {
        IN_HandleInactiveInput(&controller->buttons, ovrButton_X, "X", 0, qfalse);
    }

    if (buttons & ovrButton_Y) {
        IN_HandleActiveInput(&controller->buttons, ovrButton_Y, "Y", 0, qfalse);
    } else {
        IN_HandleInactiveInput(&controller->buttons, ovrButton_Y, "Y", 0, qfalse);
    }
}

void IN_VRInputFrame( void )
{
	if (controllerInit == qfalse) {
		memset(&leftController, 0, sizeof(leftController));
		memset(&rightController, 0, sizeof(rightController));
		controllerInit = qtrue;
	}
	engine_t* engine = VR_GetEngine();

	if (vr_extralatencymode != NULL &&
            vr_extralatencymode->integer) {
        //TODO:vrapi_SetExtraLatencyMode(VR_GetEngine()->ovr, VRAPI_EXTRA_LATENCY_MODE_ON);
    }

	if (vr_refreshrate != NULL && vr_refreshrate->integer) {
        //TODO:OXR(engine->appState.pfnRequestDisplayRefreshRate(engine->appState.Session, (float)vr_refreshrate->integer));
	}

	vr.virtual_screen = VR_useScreenLayer();

    VR_processHaptics();

	//trigger frame tick for haptics
    //VR_HapticEvent("frame_tick", 0, 0, 0, 0, 0);

    //button mapping
    uint32_t lButtons = 0;
    //if (GetActionStateBoolean(menuClickPath, SIDE_LEFT).currentState) lButtons |= ovrButton_Enter;
    if (GetActionStateBoolean(XAction, SIDE_LEFT).currentState) lButtons |= ovrButton_X;
    if (GetActionStateBoolean(YAction, SIDE_LEFT).currentState) lButtons |= ovrButton_Y;
    if (GetActionStateBoolean(sideAction, SIDE_LEFT).currentState) lButtons |= ovrButton_GripTrigger;
    if (GetActionStateBoolean(touchpadAction, SIDE_LEFT).currentState) lButtons |= ovrButton_LThumb;
    IN_VRButtons(qfalse, lButtons);
    uint32_t rButtons = 0;
    if (GetActionStateBoolean(AAction, SIDE_RIGHT).currentState) rButtons |= ovrButton_A;
    if (GetActionStateBoolean(BAction, SIDE_RIGHT).currentState) rButtons |= ovrButton_B;
    if (GetActionStateBoolean(sideAction, SIDE_RIGHT).currentState) rButtons |= ovrButton_GripTrigger;
    if (GetActionStateBoolean(touchpadAction, SIDE_RIGHT).currentState) rButtons |= ovrButton_RThumb;
    IN_VRButtons(qtrue, rButtons);

    //index finger click
    XrActionStateFloat indexState;
    indexState = GetActionStateFloat(triggerAction, SIDE_LEFT);
    IN_VRTriggers(qfalse, indexState.currentState);
    indexState = GetActionStateFloat(triggerAction, SIDE_RIGHT);
    IN_VRTriggers(qtrue, indexState.currentState);

    //thumbstick
    XrActionStateVector2f moveJoystickState;
    moveJoystickState = GetActionStateVector2(joystickAction, SIDE_LEFT);
    IN_VRJoystick(qfalse, moveJoystickState.currentState.x, moveJoystickState.currentState.y);
    moveJoystickState = GetActionStateVector2(joystickAction, SIDE_RIGHT);
    IN_VRJoystick(qtrue, moveJoystickState.currentState.x, moveJoystickState.currentState.y);

	lastframetime = in_vrEventTime;
	in_vrEventTime = Sys_Milliseconds( );
}

void IN_VRSyncActions( void )
{
    if (!inputInitialized)
    {
        InitializeActions();
        inputInitialized = qtrue;
    }

    XrActiveActionSet activeActionSet = {};
    activeActionSet.actionSet = actionSet;
    activeActionSet.subactionPath = XR_NULL_PATH;
    XrActionsSyncInfo syncInfo;
    syncInfo.type = XR_TYPE_ACTIONS_SYNC_INFO;
    syncInfo.countActiveActionSets = 1;
    syncInfo.activeActionSets = &activeActionSet;
    CHECK_XRCMD(xrSyncActions(VR_GetEngine()->appState.Session, &syncInfo));
}

void IN_VRUpdateControllers( float predictedDisplayTime )
{
    engine_t* engine = VR_GetEngine();

    //get controller poses
    for (int i = 0; i < 2; i++) {
        XrSpaceLocation loc = {};
        loc.type = XR_TYPE_SPACE_LOCATION;
        XrResult res = xrLocateSpace(aimSpace[i], engine->appState.CurrentSpace, predictedDisplayTime, &loc);
        if (res != XR_SUCCESS) {
            Com_Printf("xrLocateSpace error: %d", (int)res);
        }

        engine->appState.TrackedController[i].Active = (loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0;
        engine->appState.TrackedController[i].Pose = loc.pose;
    }

    //apply controller poses
    if (engine->appState.TrackedController[0].Active)
        IN_VRController(qfalse, engine->appState.TrackedController[0].Pose);
    if (engine->appState.TrackedController[1].Active)
        IN_VRController(qtrue, engine->appState.TrackedController[1].Pose);
}

void IN_VRUpdateHMD( XrPosef xfStageFromHead )
{
    // We extract Yaw, Pitch, Roll instead of directly using the orientation
    // to allow "additional" yaw manipulation with mouse/controller.
    const XrQuaternionf quatHmd = xfStageFromHead.orientation;
    const XrVector3f positionHmd = xfStageFromHead.position;
    vec3_t rotation = {0, 0, 0};
    QuatToYawPitchRoll(quatHmd, rotation, vr.hmdorientation);
    VectorSet(vr.hmdposition, positionHmd.x, positionHmd.y + vr_heightAdjust->value, positionHmd.z);

    //Position
    VectorSubtract(vr.hmdposition_last, vr.hmdposition, vr.hmdposition_delta);

    //Keep this for our records
    VectorCopy(vr.hmdposition, vr.hmdposition_last);

    //Orientation
    VectorSubtract(vr.hmdorientation_last, vr.hmdorientation, vr.hmdorientation_delta);

    //Keep this for our records
    VectorCopy(vr.hmdorientation, vr.hmdorientation_last);

    // View yaw delta
    const float clientview_yaw = vr.clientviewangles[YAW] - vr.hmdorientation[YAW];
    vr.clientview_yaw_delta = vr.clientview_yaw_last - clientview_yaw;
    vr.clientview_yaw_last = clientview_yaw;
}

//#endif
