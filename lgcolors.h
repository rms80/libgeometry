// Copyright Ryan Schmidt 2011.
// Distributed under the Boost Software License, Version 1.0.
// (See copy at http://www.boost.org/LICENSE_1_0.txt)
#define VIDEO_COLORS



#ifdef VIDEO_COLORS

static const float lgBLACK[3] = {0.08f,0.08f,0.08f};
static const float lgWHITE[3] = {0.97f,0.97f,0.97f};

static const float lgRED[3] = {0.83f, 0.07f, 0.07f};
static const float lgGREEN[3] = {0.14f,0.9f,0.14f};
static const float lgBLUE[3] = {0.03f,0.03f,0.78f};


#else

static const float lgBLACK[3] = {0,0,0};
static const float lgWHITE[3] = {1,1,1};

static const float lgRED[3] = {1,0,0};
static const float lgGREEN[3] = {0,1,0};
static const float lgBLUE[3] = {0,0,1};


#endif