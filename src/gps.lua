-- not full map
-- no auto clear blip
-- not in vehicle only

-- this is just a fun project

local ffi    = require("ffi")
local memory = require("memory")
local hook   = require("monethook")

local BASE = MONET_GTASA_BASE

local MAX_NODES  = 2000
local GPS_WIDTH  = 6.0
local GPS_COLOR  = 0xFF1818B4

local rwTRISTRIP = 4
local rwTEX_RAST = 1

local GOT_THEPATHS = BASE + 0x677378
local GOT_GMM      = BASE + 0x679CCC
local GOT_MSRT     = BASE + 0x6773CC
local GOT_RSG      = BASE + 0x67910C

local MM_TBLIP   = 0x48
local MM_ZOOM    = 0x58
local MM_DRAWMAP = 0x6C

local RT_SIZE = 0x28
local RT_POS  = 0x08
local RT_CTR  = 0x14
local RT_DISP = 0x26

local PF_NODES = 0x804
local PN_SIZE  = 0x1C
local PN_POS   = 0x08

ffi.cdef[[
typedef struct { float x, y, z; }    CVector;
typedef struct { float x, y; }       CVector2D;
typedef struct { short areaId; short nodeId; } CNodeAddress;
typedef struct { float x, y, z, rhw; uint32_t color; float u, v; } RwIm2DVertex;
typedef struct { float x, y, z, rhw; unsigned int color; float u, v; } RwOpenGLVertex;

void*   _Z13FindPlayerPedi(int n);
CVector _Z15FindPlayerCoorsi(int n);

void _ZN9CPathFind12DoPathSearchEh7CVector12CNodeAddressS0_PS1_PsiPffS2_fbS1_bb(
    void* thiz, uint8_t graphType,
    CVector startCoors, CNodeAddress startNode,
    CVector targetCoors, CNodeAddress* pNodeList,
    int16_t* pNumNodes, int32_t numReq,
    float* pDist, float cutoff,
    CNodeAddress* pGivenTarget, float maxDist,
    bool noWrongWay, CNodeAddress avoid,
    bool amphibious, bool boat
);

void  _ZN6CRadar32TransformRadarPointToScreenSpaceER9CVector2DRKS0_(CVector2D* scr, CVector2D* rad);
void  _ZN6CRadar35TransformRealWorldPointToRadarSpaceER9CVector2DRKS0_(CVector2D* rad, CVector* world);
void  _ZN6CRadar15LimitRadarPointER9CVector2D(CVector2D* rad);
void  _ZN6CRadar10LimitToMapEPfS0_(float* x, float* y);
float _ZN6CWorld19FindGroundZForCoordEff(float x, float y);
void  _ZN6CRadar12DrawRadarMapEv(void* thiz);

bool _Z16RwRenderStateSet13RwRenderStatePv(uint32_t state, void* value);
bool _Z28RwIm2DRenderPrimitive_BUGFIX15RwPrimitiveTypeP14RwOpenGLVertexi(uint32_t primType, RwIm2DVertex* verts, int32_t count);
]]

local gtasa = ffi.load("GTASA")

local function gmm()  return memory.getuint32(GOT_GMM)      end
local function gtp()  return memory.getuint32(GOT_THEPATHS) end
local function grt()  return memory.getuint32(GOT_MSRT)     end
local function grsg() return memory.getuint32(GOT_RSG)      end

local function ValidBlipHandle(h)
    if h == 0 then return false end
    local idx = bit.band(h, 0xFFFF)
    local ctr = bit.rshift(h, 16)
    local tr  = grt() + idx * RT_SIZE
    return memory.getuint16(tr + RT_CTR) == ctr
       and bit.band(memory.getuint8(tr + RT_DISP), 0x3) ~= 0
end

local function GetPathNode(areaId, nodeId)
    local arr = memory.getuint32(gtp() + PF_NODES + areaId * 4)
    return arr ~= 0 and (arr + nodeId * PN_SIZE) or nil
end

local function GetNodeCoors(np)
    local p = ffi.cast("int16_t*", np + PN_POS)
    return p[0] / 8.0, p[1] / 8.0, p[2] / 8.0
end

local function Setup2dVert(v, x, y)
    v.x = x; v.y = y; v.z = 0.0001; v.rhw = 1.0
    v.color = GPS_COLOR; v.u = 0.0; v.v = 0.0
end

--  circle clip di radar space (unit circle radius=1) 
local function clipSegCircle(x0, y0, x1, y1)
    local dx, dy = x1-x0, y1-y0
    local a = dx*dx + dy*dy
    if a < 1e-10 then
        return (x0*x0 + y0*y0 <= 1.0) and x0, y0, x1, y1 or nil
    end
    local b    = 2*(x0*dx + y0*dy)
    local c    = x0*x0 + y0*y0 - 1.0
    local disc = b*b - 4*a*c
    local t0, t1 = 0.0, 1.0
    if disc >= 0 then
        local sd = math.sqrt(disc)
        local ta = (-b - sd) / (2*a)
        local tb = (-b + sd) / (2*a)
        if ta > t0 then t0 = ta end
        if tb < t1 then t1 = tb end
        if t0 > t1 then return nil end
    else
        if c > 0 then return nil end
    end
    return x0+t0*dx, y0+t0*dy,
           x0+t1*dx, y0+t1*dy
end

local resultNodes = ffi.new("CNodeAddress[2000]")
local nodePoints  = ffi.new("CVector2D[2000]")
local nodeRad     = ffi.new("CVector2D[2000]")
local lineVerts   = ffi.new("RwIm2DVertex[8000]")
local nullNode    = ffi.new("CNodeAddress")
local tmpRad      = ffi.new("CVector2D")
local tmpWorld    = ffi.new("CVector")
local clipR0      = ffi.new("CVector2D")
local clipR1      = ffi.new("CVector2D")
local clipS0      = ffi.new("CVector2D")
local clipS1      = ffi.new("CVector2D")
local outCount    = ffi.new("int16_t[1]")
local outDist     = ffi.new("float[1]")

local gpsShown    = false
local gpsDistance = 0.0

local addrRadar = tonumber(ffi.cast("uintptr_t", ffi.cast("void*",
    gtasa._ZN6CRadar12DrawRadarMapEv)))

assert(addrRadar ~= 0, "DrawRadarMap addr is 0!")

local hkRadar
hkRadar = hook.new("void(__cdecl*)(void*)", function(thiz)
    hkRadar(thiz)
    gpsShown = false

    local playa = gtasa._Z13FindPlayerPedi(0)
    if playa == nil then return end

    local mm    = gmm()
    local tblip = memory.getint32(mm + MM_TBLIP)
    if not ValidBlipHandle(tblip) then return end

    local idx = bit.band(tblip, 0xFFFF)
    local tr  = grt() + idx * RT_SIZE
    local bx  = ffi.cast("float*", tr + RT_POS)[0]
    local by  = ffi.cast("float*", tr + RT_POS)[1]
    local bz  = gtasa._ZN6CWorld19FindGroundZForCoordEff(bx, by)

    local sc = gtasa._Z15FindPlayerCoorsi(0)
    local dc = ffi.new("CVector", {x=bx, y=by, z=bz})

    outCount[0] = 0
    outDist[0]  = 0.0

    gtasa._ZN9CPathFind12DoPathSearchEh7CVector12CNodeAddressS0_PS1_PsiPffS2_fbS1_bb(
        ffi.cast("void*", gtp()),
        0, sc, nullNode, dc,
        resultNodes, outCount, MAX_NODES,
        outDist, 999999.0, nil, 999999.0,
        false, nullNode, false, false
    )

    local nc    = outCount[0]
    gpsDistance = outDist[0]
    if nc <= 0 then return end

    local bMap = memory.getuint8(mm + MM_DRAWMAP) ~= 0
    local zoom = memory.getfloat(mm + MM_ZOOM)
    local scrH = memory.getint32(grsg() + 8)

    for i = 0, nc - 1 do
        local np = GetPathNode(resultNodes[i].areaId, resultNodes[i].nodeId)
        if np then
            local nx, ny = GetNodeCoors(np)
            tmpWorld.x = nx; tmpWorld.y = ny; tmpWorld.z = 0.0
            gtasa._ZN6CRadar35TransformRealWorldPointToRadarSpaceER9CVector2DRKS0_(tmpRad, tmpWorld)
            nodeRad[i].x = tmpRad.x
            nodeRad[i].y = tmpRad.y
            if not bMap then
                gtasa._ZN6CRadar32TransformRadarPointToScreenSpaceER9CVector2DRKS0_(nodePoints + i, tmpRad)
            else
                gtasa._ZN6CRadar15LimitRadarPointER9CVector2D(tmpRad)
                gtasa._ZN6CRadar32TransformRadarPointToScreenSpaceER9CVector2DRKS0_(nodePoints + i, tmpRad)
                nodePoints[i].x = nodePoints[i].x * scrH / 448.0
                nodePoints[i].y = nodePoints[i].y * scrH / 448.0
                local px = ffi.cast("float*", nodePoints + i)
                gtasa._ZN6CRadar10LimitToMapEPfS0_(px, px + 1)
            end
        end
    end

    local vi  = 0
    local PI2 = math.pi * 0.5
    for i = 0, nc - 2 do
        local p0x, p0y, p1x, p1y
        if not bMap then
            local cx0, cy0, cx1, cy1 = clipSegCircle(
                nodeRad[i].x,   nodeRad[i].y,
                nodeRad[i+1].x, nodeRad[i+1].y)
            if not cx0 then goto skipSeg end
            clipR0.x = cx0; clipR0.y = cy0
            clipR1.x = cx1; clipR1.y = cy1
            gtasa._ZN6CRadar32TransformRadarPointToScreenSpaceER9CVector2DRKS0_(clipS0, clipR0)
            gtasa._ZN6CRadar32TransformRadarPointToScreenSpaceER9CVector2DRKS0_(clipS1, clipR1)
            p0x, p0y = clipS0.x, clipS0.y
            p1x, p1y = clipS1.x, clipS1.y
        else
            p0x, p0y = nodePoints[i].x,   nodePoints[i].y
            p1x, p1y = nodePoints[i+1].x, nodePoints[i+1].y
        end
        local dx  = p1x - p0x
        local dy  = p1y - p0y
        local ang = math.atan2(dy, dx)
        local lw  = GPS_WIDTH
        if bMap then
            local mp = math.max(140.0, math.min(960.0, zoom - 140.0))
            lw = lw * (mp / 960.0 + 0.4)
        end
        local s0x = math.cos(ang - PI2) * lw
        local s0y = math.sin(ang - PI2) * lw
        Setup2dVert(lineVerts[vi+0], p0x + s0x, p0y + s0y)
        Setup2dVert(lineVerts[vi+1], p1x + s0x, p1y + s0y)
        Setup2dVert(lineVerts[vi+2], p0x - s0x, p0y - s0y)
        Setup2dVert(lineVerts[vi+3], p1x - s0x, p1y - s0y)
        vi = vi + 4
        ::skipSeg::
    end

    if vi == 0 then return end

    gtasa._Z16RwRenderStateSet13RwRenderStatePv(rwTEX_RAST, nil)
    gtasa._Z28RwIm2DRenderPrimitive_BUGFIX15RwPrimitiveTypeP14RwOpenGLVertexi(rwTRISTRIP, lineVerts, vi)

    local n0 = GetPathNode(resultNodes[0].areaId, resultNodes[0].nodeId)
    if n0 then
        local nx, ny, nz = GetNodeCoors(n0)
        local dx, dy, dz = sc.x-nx, sc.y-ny, sc.z-nz
        gpsDistance = gpsDistance + math.sqrt(dx*dx + dy*dy + dz*dz)
    end
    gpsShown = true
end, addrRadar)

function main()
    wait(-1)
end
