// Extension lib defines
#define LIB_NAME "detour"
#define MODULE_NAME "detour"

// include the Defold SDK
#include <dmsdk/sdk.h>

#include "Detour/DetourCommon.h"
#include "Detour/DetourNavMesh.h"
#include "Detour/DetourNavMeshQuery.h"
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

struct NavMesh {
    dtNavMesh* mesh;
    dtNavMeshQuery* query;
    dtQueryFilter* filter;
};

vector<NavMesh*> meshes;

static int Delete(lua_State* L) {
    lua_getfield(L, 1, "instance");
    NavMesh* instance = (NavMesh*)lua_touserdata(L, -1);

    dtFreeNavMesh(instance->mesh);
    dtFreeNavMeshQuery(instance->query);
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

    float polyPickExt[3] = {2, 4, 2};

    
    dtPolyRef startRef;
    dtPolyRef endRef;
    dtPolyRef polys[MAX_POLYS];
    int npolys;

    instance->query->findNearestPoly(start, polyPickExt, instance->filter, &startRef, 0);
    instance->query->findNearestPoly(finish, polyPickExt, instance->filter, &endRef, 0);

    instance->query->findPath(startRef, endRef, start, finish, instance->filter, polys, &npolys, MAX_POLYS);

    dmLogInfo("npolys: %d", npolys);

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

        dmLogInfo("nstraightPath: %d", nstraightPath);

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

static int Load(lua_State* L) {
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
        {"delete", Delete},
        {0, 0}
    };
    luaL_register(L, NULL, f);
    
    return 1;
}


// Functions exposed to Lua
static const luaL_reg Module_methods[] =
{
    {"load", Load},
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
