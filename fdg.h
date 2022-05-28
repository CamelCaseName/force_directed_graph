#pragma once
#include <cstdint>
#include <malloc.h>
#include <math.h>
#include <stdio.h>

extern "C" {
	__declspec(dllexport) void __stdcall do_graph_physics(int32_t* x, int32_t* y, int32_t* mass, int32_t count, int32_t* node_1, int32_t* node_2, int32_t node_connection_count);
}