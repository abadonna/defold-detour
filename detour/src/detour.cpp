// Extension lib defines
#define LIB_NAME "detour"
#define MODULE_NAME "detour"

// include the Defold SDK
#include <dmsdk/sdk.h>

#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCrowd.h"
#include <vector>

using namespace dmVMath;
using namespace std;

static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader
{
    int magic;
    int version;
    int numTiles;
    dtNavMeshParams params;
};

struct NavMeshTileHeader
{
    dtTileRef tileRef;
    int dataSize;
};

enum SamplePolyAreas
{
    SAMPLE_POLYAREA_GROUND,
    SAMPLE_POLYAREA_WATER,
    SAMPLE_POLYAREA_ROAD,
    SAMPLE_POLYAREA_DOOR,
    SAMPLE_POLYAREA_GRASS,
    SAMPLE_POLYAREA_JUMP
};

enum SamplePolyFlags
{
    SAMPLE_POLYFLAGS_WALK		= 0x01,		// Ability to walk (ground, grass, road)
    SAMPLE_POLYFLAGS_SWIM		= 0x02,		// Ability to swim (water).
    SAMPLE_POLYFLAGS_DOOR		= 0x04,		// Ability to move through doors.
    SAMPLE_POLYFLAGS_JUMP		= 0x08,		// Ability to jump.
    SAMPLE_POLYFLAGS_DISABLED	= 0x10,		// Disabled polygon
    SAMPLE_POLYFLAGS_ALL		= 0xffff	// All abilities.
};

static const int MAX_POLYS = 256;
static const int MAX_AGENTS = 128;

struct NavMesh {
    dtNavMesh* mesh;
    dtNavMeshQuery* query;
    dtQueryFilter* filter;
    dtCrowd* crowd;
};

vector<NavMesh*> meshes;

float polyPickExt[3] = {2, 4, 2};

static int Delete(lua_State* L) {
    lua_getfield(L, 1, "instance");
    NavMesh* instance = (NavMesh*)lua_touserdata(L, -1);

    dtFreeNavMesh(instance->mesh);
    dtFreeNavMeshQuery(instance->query);
    dtFreeCrowd(instance->crowd);
    
    delete instance->filter;
   
    meshes.erase(find(meshes.begin(), meshes.end(), instance));
    delete instance;
    return 0;
}

static int FindPath(lua_State* L) {
    int count = lua_gettop(L);
    
    lua_getfield(L, 1, "instance");
    NavMesh* instance = (NavMesh*)lua_touserdata(L, -1);

    Vector3* vs = dmScript::CheckVector3(L, 2);
    Vector3* vf = dmScript::CheckVector3(L, 3);

    int straightPathOptions = (count > 3) ? luaL_checknumber(L, 4) : 0;

    float start[3] = {vs->getX(), vs->getY(), vs->getZ()};
    float finish[3] = {vf->getX(), vf->getY(), vf->getZ()};

    dtPolyRef startRef;
    dtPolyRef endRef;
    dtPolyRef polys[MAX_POLYS];
    int npolys;

    instance->query->findNearestPoly(start, polyPickExt, instance->filter, &startRef, 0);
    instance->query->findNearestPoly(finish, polyPickExt, instance->filter, &endRef, 0);

    instance->query->findPath(startRef, endRef, start, finish, instance->filter, polys, &npolys, MAX_POLYS);

    lua_newtable(L);

    if (npolys) {
        float straightPath[MAX_POLYS * 3];
        unsigned char straightPathFlags[MAX_POLYS];
        dtPolyRef straightPathPolys[MAX_POLYS];
        int nstraightPath;

        // In case of partial path, make sure the end point is clamped to the last polygon.
        float epos[3];
        dtVcopy(epos, finish);
        if (polys[npolys-1] != endRef)
        instance->query->closestPointOnPoly(polys[npolys-1], finish, epos, 0);

        instance->query->findStraightPath(start, epos, polys, npolys,
            straightPath, straightPathFlags,
            straightPathPolys, &nstraightPath, MAX_POLYS, straightPathOptions);

        if (nstraightPath) {
            for (int i = 0; i < nstraightPath; ++i) {
                lua_pushnumber(L, i);
                lua_newtable(L);
                lua_pushstring(L, "position");
                int idx = i * 3;
                dmScript::PushVector3(L, Vector3(straightPath[idx], straightPath[idx + 1], straightPath[idx + 2]));
                lua_settable(L, -3);

                lua_pushstring(L, "area");
                unsigned char area;
                instance->mesh->getPolyArea(straightPathPolys[i], &area);
                lua_pushnumber(L, area);
                lua_settable(L, -3);

                if (straightPathFlags[i] == DT_STRAIGHTPATH_START) {
                    lua_pushstring(L, "start");
                    lua_pushboolean(L, true);
                    lua_settable(L, -3);
                } else if (straightPathFlags[i] == DT_STRAIGHTPATH_END) {
                    lua_pushstring(L, "end");
                    lua_pushboolean(L, true);
                    lua_settable(L, -3);
                } else if (straightPathFlags[i] == DT_STRAIGHTPATH_OFFMESH_CONNECTION) {
                    lua_pushstring(L, "offmesh");
                    lua_pushboolean(L, true);
                    lua_settable(L, -3);
                }

                lua_settable(L, -3);
            }
        }
    }

    return 1;
}

static int SetAreaCost(lua_State* L) {
    lua_getfield(L, 1, "instance");
    NavMesh* instance = (NavMesh*)lua_touserdata(L, -1);

    for (int i = 0; i < lua_objlen(L, 2); i++) {
        lua_rawgeti(L, 2, i + 1);
        float cost = luaL_checknumber(L, -1);
        instance->filter->setAreaCost(i, cost);
    }

    return 0;
}

static int SetIncludeFlags(lua_State* L) {
    lua_getfield(L, 1, "instance");
    NavMesh* instance = (NavMesh*)lua_touserdata(L, -1);

    unsigned short flags = luaL_checknumber(L, 2);
    instance->filter->setIncludeFlags(flags);

    return 0;
}

static int AddAgent(lua_State* L) {
    lua_getfield(L, 1, "instance");
    NavMesh* instance = (NavMesh*)lua_touserdata(L, -1);
    
    Vector3* p = dmScript::CheckVector3(L, 2);
    float pos[3] = {p->getX(), p->getY(), p->getZ()};

    dtCrowdAgentParams ap;
    memset(&ap, 0, sizeof(ap));
    ap.radius = luaL_checknumber(L, 3);
    ap.height = luaL_checknumber(L, 4);
    ap.maxAcceleration = luaL_checknumber(L, 5);
    ap.maxSpeed = luaL_checknumber(L, 6);
    
    ap.collisionQueryRange = ap.radius * 12.0f;
    ap.pathOptimizationRange = ap.radius * 30.0f;

    ap.updateFlags = luaL_checknumber(L, 7);   
    ap.obstacleAvoidanceType = 3; //high
    //ap.separationWeight = m_toolParams.m_separationWeight;

    int idx = instance->crowd->addAgent(pos, &ap);
    lua_pushnumber(L, idx);
    
    return 1;
}

static int RemoveAgent(lua_State* L) {
    lua_getfield(L, 1, "instance");
    NavMesh* instance = (NavMesh*)lua_touserdata(L, -1);

    int idx = lua_tonumber(L, 3);
    instance->crowd->removeAgent(idx);
    
    return 0;
}

static int SetTarget(lua_State* L) {
    int count = lua_gettop(L);
    lua_getfield(L, 1, "instance");
    NavMesh* instance = (NavMesh*)lua_touserdata(L, -1);

    Vector3* p = dmScript::CheckVector3(L, 2);
    float pos[3] = {p->getX(), p->getY(), p->getZ()};

    float targetPos[3];
    dtPolyRef targetRef;
    
    instance->query->findNearestPoly(pos, polyPickExt, instance->filter, &targetRef, targetPos);

    if (count > 2) {
        int idx = lua_tonumber(L, 3);
        instance->crowd->requestMoveTarget(idx, targetRef, targetPos);
        return 0;
    }
    
    for (int i = 0; i < instance->crowd->getAgentCount(); ++i) {
        const dtCrowdAgent* ag = instance->crowd->getAgent(i);
        if (!ag->active) continue;
        instance->crowd->requestMoveTarget(i, targetRef, targetPos);
    }

    return 0;
}

static int ResetTarget(lua_State* L) {
    int count = lua_gettop(L);
    lua_getfield(L, 1, "instance");
    NavMesh* instance = (NavMesh*)lua_touserdata(L, -1);

    if (count > 1) {
        int idx = lua_tonumber(L, 2);
        instance->crowd->resetMoveTarget(idx);
        return 0;
    }

    for (int i = 0; i < instance->crowd->getAgentCount(); ++i) {
        instance->crowd->resetMoveTarget(i);
    }

    return 0;
}

static int Update(lua_State* L) {
    lua_getfield(L, 1, "instance");
    NavMesh* instance = (NavMesh*)lua_touserdata(L, -1);

    dtCrowdAgentDebugInfo agentDebug;
    instance->crowd->update(luaL_checknumber(L, 2), &agentDebug);

    lua_newtable(L);
    
    for (int i = 0; i < instance->crowd->getAgentCount(); ++i)
    {
        const dtCrowdAgent* ag = instance->crowd->getAgent(i);
        if (!ag->active) continue;

        lua_pushnumber(L, i);
        
        lua_newtable(L);
        lua_pushstring(L, "position");
        dmScript::PushVector3(L, Vector3(ag->npos[0], ag->npos[1], ag->npos[2]));
        lua_settable(L, -3);

        lua_pushstring(L, "velocity");
        dmScript::PushVector3(L, Vector3(ag->vel[0], ag->vel[1], ag->vel[2]));
        lua_settable(L, -3);

        lua_pushstring(L, "state");
        if (ag->state == DT_CROWDAGENT_STATE_WALKING) {
            lua_pushstring(L, "walk");
        } else if (ag->state == DT_CROWDAGENT_STATE_OFFMESH) {
            lua_pushstring(L, "offmesh");
        } else {
            lua_pushstring(L, "invalid");
        }
        lua_settable(L, -3);

        lua_settable(L, -3);
    }
    
    return 1;
}

static int CreateCrowd(lua_State* L) {
    lua_getfield(L, 1, "instance");
    NavMesh* instance = (NavMesh*)lua_touserdata(L, -1);

    float maxAgentRadius = luaL_checknumber(L, 2);
    if (instance->crowd == NULL) {
        instance->crowd = dtAllocCrowd();
    }

    instance->crowd->init(MAX_AGENTS, maxAgentRadius, instance->mesh);
    // Make polygons with 'disabled' flag invalid.
    instance->crowd->getEditableFilter(0)->setExcludeFlags(SAMPLE_POLYFLAGS_DISABLED);
    // Setup local avoidance params to different qualities.
    dtObstacleAvoidanceParams params;
    // Use mostly default settings, copy from dtCrowd.
    memcpy(&params, instance->crowd->getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));

    // Low (11)
    params.velBias = 0.5f;
    params.adaptiveDivs = 5;
    params.adaptiveRings = 2;
    params.adaptiveDepth = 1;
    instance->crowd->setObstacleAvoidanceParams(0, &params);

    // Medium (22)
    params.velBias = 0.5f;
    params.adaptiveDivs = 5; 
    params.adaptiveRings = 2;
    params.adaptiveDepth = 2;
    instance->crowd->setObstacleAvoidanceParams(1, &params);

    // Good (45)
    params.velBias = 0.5f;
    params.adaptiveDivs = 7;
    params.adaptiveRings = 2;
    params.adaptiveDepth = 3;
    instance->crowd->setObstacleAvoidanceParams(2, &params);

    // High (66)
    params.velBias = 0.5f;
    params.adaptiveDivs = 7;
    params.adaptiveRings = 3;
    params.adaptiveDepth = 3;

    instance->crowd->setObstacleAvoidanceParams(3, &params);

    
    lua_newtable(L);
    lua_pushstring(L, "instance");
    lua_pushlightuserdata(L, instance);
    lua_settable(L, -3);

    static const luaL_Reg f[] =
    {
        {"add_agent", AddAgent},
        {"remove_agent", RemoveAgent},
        {"set_target", SetTarget},
        {"reset_target", ResetTarget},
        {"update", Update},
        {0, 0}
    };
    luaL_register(L, NULL, f);

    return 1;
}

static int Init(lua_State* L) {
    const char* content = luaL_checkstring(L, 1);

    NavMeshSetHeader* header = (NavMeshSetHeader*)&content[0];
    content += sizeof(NavMeshSetHeader);
    
    if (header->magic != NAVMESHSET_MAGIC) {
        dmLogInfo("wrong navigation mesh data file!");
    }

    dmLogInfo("nun tiles: %d", header->numTiles);

    NavMesh* instance = new NavMesh;

    instance->mesh = dtAllocNavMesh();
    instance->mesh->init(&header->params);

    // Read tiles.
    for (int i = 0; i < header->numTiles; ++i)
    {
        NavMeshTileHeader* tileHeader = (NavMeshTileHeader*)content;
        content += sizeof(NavMeshTileHeader);

        dmLogInfo("tile ref: %d", tileHeader->tileRef);
        dmLogInfo("size: %d", tileHeader->dataSize);

        if (!tileHeader->tileRef || !tileHeader->dataSize) break;

        unsigned char* data = (unsigned char*)dtAlloc(tileHeader->dataSize, DT_ALLOC_PERM);
        if (!data) break;

        memset(data, 0, tileHeader->dataSize);
        memcpy(data, content, tileHeader->dataSize);
        content += tileHeader->dataSize;
       
        instance->mesh->addTile(data, tileHeader->dataSize, DT_TILE_FREE_DATA, tileHeader->tileRef, 0);
    }

    instance->query = dtAllocNavMeshQuery();
    instance->query->init(instance->mesh, 2048);

    dtQueryFilter* filter = new dtQueryFilter();

    filter->setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f);
    filter->setAreaCost(SAMPLE_POLYAREA_WATER, 10.0f);
    filter->setAreaCost(SAMPLE_POLYAREA_ROAD, 1.0f);
    filter->setAreaCost(SAMPLE_POLYAREA_DOOR, 1.0f);
    filter->setAreaCost(SAMPLE_POLYAREA_GRASS, 2.0f);
    filter->setAreaCost(SAMPLE_POLYAREA_JUMP, 1.5f);

    filter->setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
    filter->setExcludeFlags(0);

    instance->filter = filter;
    instance->crowd = NULL;

    meshes.push_back(instance);

    lua_newtable(L);
    lua_pushstring(L, "instance");
    lua_pushlightuserdata(L, instance);
    lua_settable(L, -3);

    static const luaL_Reg f[] =
    {
        {"find_path", FindPath},
        {"set_area_cost", SetAreaCost},
        {"set_include_flags", SetIncludeFlags},
        {"create_crowd", CreateCrowd},
        {"delete", Delete},
        {0, 0}
    };
    luaL_register(L, NULL, f);
    
    return 1;
}


// Functions exposed to Lua
static const luaL_reg Module_methods[] =
{
    {"init", Init},
    {0, 0}
};

static void LuaInit(lua_State* L)
{
    int top = lua_gettop(L);

    // Register lua names
    luaL_register(L, MODULE_NAME, Module_methods);

    lua_pop(L, 1);
    assert(top == lua_gettop(L));
}

static dmExtension::Result AppInitializeMyExtension(dmExtension::AppParams* params)
{
    dmLogInfo("Detour initialized");
    return dmExtension::RESULT_OK;
}

static dmExtension::Result InitializeMyExtension(dmExtension::Params* params)
{
    // Init Lua
    LuaInit(params->m_L);
    dmLogInfo("Registered %s Extension", MODULE_NAME);
    return dmExtension::RESULT_OK;
}

static dmExtension::Result AppFinalizeMyExtension(dmExtension::AppParams* params)
{
    return dmExtension::RESULT_OK;
}

static dmExtension::Result FinalizeMyExtension(dmExtension::Params* params)
{
    return dmExtension::RESULT_OK;
}

static dmExtension::Result OnUpdateMyExtension(dmExtension::Params* params)
{
    return dmExtension::RESULT_OK;
}

static void OnEventMyExtension(dmExtension::Params* params, const dmExtension::Event* event)
{
    switch(event->m_Event)
    {
        case dmExtension::EVENT_ID_ACTIVATEAPP:
            dmLogInfo("OnEventMyExtension - EVENT_ID_ACTIVATEAPP");
            break;
        case dmExtension::EVENT_ID_DEACTIVATEAPP:
            dmLogInfo("OnEventMyExtension - EVENT_ID_DEACTIVATEAPP");
            break;
        case dmExtension::EVENT_ID_ICONIFYAPP:
            dmLogInfo("OnEventMyExtension - EVENT_ID_ICONIFYAPP");
            break;
        case dmExtension::EVENT_ID_DEICONIFYAPP:
            dmLogInfo("OnEventMyExtension - EVENT_ID_DEICONIFYAPP");
            break;
        default:
            //dmLogWarning("OnEventMyExtension - Unknown event id");
            break;
    }
}

// Defold SDK uses a macro for setting up extension entry points:
//
// DM_DECLARE_EXTENSION(symbol, name, app_init, app_final, init, update, on_event, final)

// MyExtension is the C++ symbol that holds all relevant extension data.
// It must match the name field in the `ext.manifest`
DM_DECLARE_EXTENSION(detour, LIB_NAME, AppInitializeMyExtension, AppFinalizeMyExtension, InitializeMyExtension, OnUpdateMyExtension, OnEventMyExtension, FinalizeMyExtension)
