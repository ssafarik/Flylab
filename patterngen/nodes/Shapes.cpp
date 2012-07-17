// ****************************************
// * (C)opyright 2001, Steve Safarik
// ****************************************



// ****************************************
// * Includes
// ****************************************

#include "stdafx.h"
#include "GameTypes.h"
#include "Shapes.h"



// ****************************************
// * Module Data
// ****************************************

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


// How much rotation happens automatically.
#define AUTOROTATE_SUN			 0.2
#define AUTOROTATE_SHIP			 0.0
#define AUTOROTATE_ROCK			 0.10
#define AUTOROTATE_BULLET		 0.5
#define AUTOROTATE_METER		 0.0
#define AUTOROTATE_DEBRIS		 0.12
#define AUTOROTATE_NUMERAL		 0.0
#define AUTOROTATE_LETTER		 0.0



// ******************************
// saShapes[]
// - Table containing the point data for all the various ships, bullets, etc.
// 
SHAPE	saShapes[NSHAPES] =
{
	// A = Undamaged
	// B = Right-engine gone, left intact, 
	// C = Left-engine gone, right intact, 
	// D = Both engines gone

	// ShapeType,   fInteract, fInvincible; Autorotation;  cockpit,          left,             right;            distancehit,   nPoints, list of points

	// Sun A,B,C,D						'~'
	//spiral     {ShapeType_Ship,   TRUE,  TRUE,   AUTOROTATE_SUN,     dptHITPOINT_SUN1,    dptHITPOINT_SUN2,    dptHITPOINT_SUN3,    DISTANCEHIT_SUN,    14, {{ 0.0,  0.0}, {-1.0,  0.0}, {-1.0,  1.0}, { 1.0,  1.0}, { 1.0, -1.0}, {-3.0, -1.0}, {-3.0,  3.0}, { 3.0,  3.0}, { 3.0, -3.0}, {-5.0, -3.0}, {-5.0,  5.0}, { 5.0,  5.0}, { 5.0, -5.0}, {-5.0, -5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	//4-square   {ShapeType_Ship,   TRUE,  TRUE,   AUTOROTATE_SUN,     dptHITPOINT_SUN1,    dptHITPOINT_SUN2,    dptHITPOINT_SUN3,    DISTANCEHIT_SUN,     5, {{-5.0,  0.0}, { 0.0,  4.0}, { 5.0,  0.0}, { 0.0, -4.0}, {-5.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	//5-star     {ShapeType_Ship,	TRUE,  TRUE,   AUTOROTATE_SUN,     dptHITPOINT_SUN1,    dptHITPOINT_SUN2,    dptHITPOINT_SUN3,    DISTANCEHIT_SUN,     6, {{ 0.0, -5.0}, { 2.9,  4.0}, {-4.8, -1.5}, { 4.8, -1.5}, {-2.9,  4.0}, { 0.0, -5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	//Null sign  {ShapeType_Ship,	TRUE,  TRUE,   AUTOROTATE_SUN,     dptHITPOINT_SUN1,    dptHITPOINT_SUN2,    dptHITPOINT_SUN3,    DISTANCEHIT_SUN,    12, {{ 0.0,  6.0}, { 0.0,  3.5}, { 2.0,  3.5}, { 4.0,  0.0}, { 2.0, -3.5}, { 0.0, -3.5}, { 0.0,  3.5}, {-2.0,  3.5}, {-4.0,  0.0}, {-2.0, -3.5}, { 0.0, -3.5}, { 0.0, -6.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	
		TRUE,  
		TRUE,   
		AUTOROTATE_SUN,     
		dptHITPOINT_SUN1,    
		dptHITPOINT_SUN2,    
		dptHITPOINT_SUN3,    
		DISTANCEHIT_SUN,     
		6, 
	{{ 0.0, -5.0}, { 2.9,  4.0}, {-4.8, -1.5}, { 4.8, -1.5}, {-2.9,  4.0}, { 0.0, -5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  TRUE,   AUTOROTATE_SUN,     dptHITPOINT_SUN1,    dptHITPOINT_SUN2,    dptHITPOINT_SUN3,    DISTANCEHIT_SUN,     6, {{ 0.0, -5.0}, { 2.9,  4.0}, {-4.8, -1.5}, { 4.8, -1.5}, {-2.9,  4.0}, { 0.0, -5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  TRUE,   AUTOROTATE_SUN,     dptHITPOINT_SUN1,    dptHITPOINT_SUN2,    dptHITPOINT_SUN3,    DISTANCEHIT_SUN,     6, {{ 0.0, -5.0}, { 2.9,  4.0}, {-4.8, -1.5}, { 4.8, -1.5}, {-2.9,  4.0}, { 0.0, -5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  TRUE,   AUTOROTATE_SUN,     dptHITPOINT_SUN1,    dptHITPOINT_SUN2,    dptHITPOINT_SUN3,    DISTANCEHIT_SUN,     6, {{ 0.0, -5.0}, { 2.9,  4.0}, {-4.8, -1.5}, { 4.8, -1.5}, {-2.9,  4.0}, { 0.0, -5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Ship1 A,B,C,D (tie-fighter)		'!'
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_LEFT,    dptHITPOINT_RIGHT,   DISTANCEHIT_SHIP,    7, {{-3.0,  2.0}, {-3.0,  5.0}, { 1.0,  1.0}, { 0.0, -5.0}, {-1.0,  1.0}, { 3.0,  5.0}, { 3.0,  2.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_LEFT,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    6, {{-3.0,  2.0}, {-3.0,  5.0}, { 1.0,  1.0}, { 0.0, -5.0}, {-1.0,  1.0}, { 3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_NONE,    dptHITPOINT_RIGHT,   DISTANCEHIT_SHIP,    6, {{-3.0,  5.0}, { 1.0,  1.0}, { 0.0, -5.0}, {-1.0,  1.0}, { 3.0,  5.0}, { 3.0,  2.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{-3.0,  5.0}, { 1.0,  1.0}, { 0.0, -5.0}, {-1.0,  1.0}, { 3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Ship2 A,B,C,D (enterprise)		'@'
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_LEFT,    dptHITPOINT_RIGHT,   DISTANCEHIT_SHIP,   15, {{ 2.0,  1.0}, { 2.0,  5.0}, { 2.0,  3.0}, { 0.0,  1.0}, { 0.0, -1.0}, { 2.0, -2.0}, { 2.0, -4.0}, { 0.0, -5.0}, {-2.0, -4.0}, {-2.0, -2.0}, { 0.0, -1.0}, { 0.0,  1.0}, {-2.0,  3.0}, {-2.0,  5.0}, {-2.0,  1.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_LEFT,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   13, {{ 2.0,  3.0}, { 0.0,  1.0}, { 0.0, -1.0}, { 2.0, -2.0}, { 2.0, -4.0}, { 0.0, -5.0}, {-2.0, -4.0}, {-2.0, -2.0}, { 0.0, -1.0}, { 0.0,  1.0}, {-2.0,  3.0}, {-2.0,  1.0}, {-2.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_NONE,    dptHITPOINT_RIGHT,   DISTANCEHIT_SHIP,   13, {{-2.0,  3.0}, { 0.0,  1.0}, { 0.0, -1.0}, { 2.0, -2.0}, { 2.0, -4.0}, { 0.0, -5.0}, {-2.0, -4.0}, {-2.0, -2.0}, { 0.0, -1.0}, { 0.0,  1.0}, { 2.0,  3.0}, { 2.0,  1.0}, { 2.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   11, {{ 2.0,  3.0}, { 0.0,  1.0}, { 0.0, -1.0}, { 2.0, -2.0}, { 2.0, -4.0}, { 0.0, -5.0}, {-2.0, -4.0}, {-2.0, -2.0}, { 0.0, -1.0}, { 0.0,  1.0}, {-2.0,  3.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Ship3 A,B,C,D (scissors)			'#'
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_LEFT,    dptHITPOINT_RIGHT,   DISTANCEHIT_SHIP,   17, {{-2.0,  4.0}, {-3.0,  4.0}, {-3.0,  5.0}, {-2.0,  5.0}, {-2.0,  4.0}, {-1.0,  2.0}, { 0.0,  2.0}, {-1.0, -5.0}, { 0.0,  2.0}, { 1.0, -5.0}, { 0.0,  2.0}, { 1.0,  2.0}, { 2.0,  4.0}, { 2.0,  5.0}, { 3.0,  5.0}, { 3.0,  4.0}, { 2.0,  4.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_LEFT,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   13, {{-2.0,  4.0}, {-3.0,  4.0}, {-3.0,  5.0}, {-2.0,  5.0}, {-2.0,  4.0}, {-1.0,  2.0}, { 0.0,  2.0}, {-1.0, -5.0}, { 0.0,  2.0}, { 1.0, -5.0}, { 0.0,  2.0}, { 1.0,  2.0}, { 2.0,  4.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_NONE,    dptHITPOINT_RIGHT,   DISTANCEHIT_SHIP,   13, {{-2.0,  4.0}, {-1.0,  2.0}, { 0.0,  2.0}, {-1.0, -5.0}, { 0.0,  2.0}, { 1.0, -5.0}, { 0.0,  2.0}, { 1.0,  2.0}, { 2.0,  4.0}, { 2.0,  5.0}, { 3.0,  5.0}, { 3.0,  4.0}, { 2.0,  4.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    9, {{-2.0,  4.0}, {-1.0,  2.0}, { 0.0,  2.0}, {-1.0, -5.0}, { 0.0,  2.0}, { 1.0, -5.0}, { 0.0,  2.0}, { 1.0,  2.0}, { 2.0,  4.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Ship4 A,B,C,D (wedge)			'$'
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_LEFT,    dptHITPOINT_RIGHT,   DISTANCEHIT_SHIP,    5, {{ 0.0,  2.0}, {-3.0,  5.0}, { 0.0, -5.0}, { 3.0,  5.0}, { 0.0,  2.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_LEFT,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{ 0.0,  2.0}, {-3.0,  5.0}, { 0.0, -5.0}, { 2.0,  2.0}, { 0.0,  2.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_NONE,    dptHITPOINT_RIGHT,   DISTANCEHIT_SHIP,    5, {{ 0.0,  2.0}, {-2.0,  2.0}, { 0.0, -5.0}, { 3.0,  5.0}, { 0.0,  2.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{ 0.0,  2.0}, {-2.0,  2.0}, { 0.0, -5.0}, { 2.0,  2.0}, { 0.0,  2.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Ship5 A,B,C,D (cube)				'%'
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_LEFT,    dptHITPOINT_RIGHT,   DISTANCEHIT_SHIP,   11, {{-3.0, -1.0}, {-3.0,  3.0}, { 1.0,  3.0}, { 1.0, -1.0}, {-3.0, -1.0}, {-1.0, -3.0}, { 3.0, -3.0}, { 1.0, -1.0}, { 1.0,  3.0}, { 3.0,  1.0}, { 3.0, -3.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_LEFT,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   12, {{-3.0, -1.0}, {-3.0,  3.0}, { 1.0,  3.0}, { 1.0, -1.0}, {-3.0, -1.0}, {-1.0, -3.0}, { 3.0, -3.0}, { 1.0, -1.0}, { 1.0,  3.0}, { 2.0,  1.5}, { 3.0,  1.0}, { 3.0, -3.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_NONE,    dptHITPOINT_RIGHT,   DISTANCEHIT_SHIP,   12, {{-3.0, -1.0}, {-2.0,  1.0}, {-3.0,  3.0}, { 1.0,  3.0}, { 1.0, -1.0}, {-3.0, -1.0}, {-1.0, -3.0}, { 3.0, -3.0}, { 1.0, -1.0}, { 1.0,  3.0}, { 3.0,  1.0}, { 3.0, -3.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   13, {{-3.0, -1.0}, {-2.0,  1.0}, {-3.0,  3.0}, { 1.0,  3.0}, { 1.0, -1.0}, {-3.0, -1.0}, {-1.0, -3.0}, { 3.0, -3.0}, { 1.0, -1.0}, { 1.0,  3.0}, { 2.0,  1.5}, { 3.0,  1.0}, { 3.0, -3.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Ship5 A,B,C,D (Heart)
	//{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_LEFT,    dptHITPOINT_RIGHT,   DISTANCEHIT_SHIP,   11, {{ 3.0,  0.0}, { 4.0,  1.0}, { 4.0,  2.0}, { 3.0,  3.0}, { 1.0,  3.0}, {-4.0,  0.0}, { 1.0, -3.0}, { 3.0, -3.0}, { 4.0, -2.0}, { 4.0, -1.0}, { 3.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	//{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_LEFT,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   11, {{ 0.0,  1.0}, { 4.0,  1.0}, { 4.0,  2.0}, { 3.0,  3.0}, { 1.0,  3.0}, {-4.0,  0.0}, { 1.0, -3.0}, { 3.0, -3.0}, { 4.0, -2.0}, { 4.0, -1.0}, { 0.0,  1.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	//{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_NONE,    dptHITPOINT_RIGHT,   DISTANCEHIT_SHIP,   14, {{ 3.0,  0.0}, { 4.0,  1.0}, { 4.0,  2.0}, { 3.0,  3.0}, { 1.0,  3.0}, {-0.6,  2.0}, { 2.0,  0.5}, {-1.5,  1.5}, {-4.0,  0.0}, { 1.0, -3.0}, { 3.0, -3.0}, { 4.0, -2.0}, { 4.0, -1.0}, { 3.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	//{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_SHIP,    dptHITPOINT_COCKPIT, dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   10, {{ 3.0,  0.0}, { 4.0,  1.0}, { 2.0,  0.5}, {-1.5,  1.5}, {-4.0,  0.0}, { 1.0, -3.0}, { 3.0, -3.0}, { 4.0, -2.0}, { 4.0, -1.0}, { 3.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Rock A,B,C,D
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_ROCK,    dptHITPOINT_ROCK1,   dptHITPOINT_ROCK2,   dptHITPOINT_ROCK3,   DISTANCEHIT_ROCK,    9, {{ 1.0, -4.0}, { 4.0, -3.0}, { 5.0,  2.0}, { 3.0,  5.0}, { 1.0,  5.0}, {-4.0,  3.0}, {-3.0, -3.0}, { 0.0, -3.0}, { 1.0, -4.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_ROCK,    dptHITPOINT_ROCK1,   dptHITPOINT_ROCK2,   dptHITPOINT_ROCK3,   DISTANCEHIT_ROCK,    9, {{ 1.0, -4.0}, { 4.0, -3.0}, { 5.0,  2.0}, { 3.0,  5.0}, { 1.0,  5.0}, {-4.0,  3.0}, {-3.0, -3.0}, { 0.0, -3.0}, { 1.0, -4.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_ROCK,    dptHITPOINT_ROCK1,   dptHITPOINT_ROCK2,   dptHITPOINT_ROCK3,   DISTANCEHIT_ROCK,    9, {{ 1.0, -4.0}, { 4.0, -3.0}, { 5.0,  2.0}, { 3.0,  5.0}, { 1.0,  5.0}, {-4.0,  3.0}, {-3.0, -3.0}, { 0.0, -3.0}, { 1.0, -4.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Ship,	TRUE,  FALSE,  AUTOROTATE_ROCK,    dptHITPOINT_ROCK1,   dptHITPOINT_ROCK2,   dptHITPOINT_ROCK3,   DISTANCEHIT_ROCK,    9, {{ 1.0, -4.0}, { 4.0, -3.0}, { 5.0,  2.0}, { 3.0,  5.0}, { 1.0,  5.0}, {-4.0,  3.0}, {-3.0, -3.0}, { 0.0, -3.0}, { 1.0, -4.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Debris A,B,C,D
	{ShapeType_Debris,	FALSE, FALSE,  AUTOROTATE_DEBRIS,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    3, {{-2.0,  0.0}, { 2.0,  0.0}, { 0.0,  2.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Debris,	FALSE, FALSE,  AUTOROTATE_DEBRIS,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    2, {{-2.5, -2.5}, { 2.5,  2.5}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Debris,	FALSE, FALSE,  AUTOROTATE_DEBRIS,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    2, {{ 0.0, -3.0}, { 0.0,  3.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Debris,	FALSE, FALSE,  AUTOROTATE_DEBRIS,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    2, {{ 2.5, -2.5}, {-2.5,  2.5}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Bullet
	//{ShapeType_Bullet,TRUE,  FALSE,  AUTOROTATE_BULLET,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    3, {{ 0.0,  1.0}, {-2.0,  0.0}, { 0.0, -1.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Bullet,	TRUE,  FALSE,  AUTOROTATE_BULLET,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    4, {{ 0.5, -1.0}, {-0.5,  0.0}, { 0.5,  0.0}, {-0.5,  1.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Meter (a horizontal line)
	{ShapeType_Symbol,	FALSE, TRUE,   AUTOROTATE_METER,   dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    2, {{-5.0,  0.0}, { 5.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Numeral 0
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_NUMERAL, dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    9, {{ 3.0, -4.0}, { 2.0, -5.0}, {-2.0, -5.0}, {-3.0, -4.0}, {-3.0,  4.0}, {-2.0,  5.0}, { 2.0,  5.0}, { 3.0,  4.0}, { 3.0, -4.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Numeral 1
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_NUMERAL, dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    2, {{ 0.0, -5.0}, { 0.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Numeral 2
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_NUMERAL, dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   10, {{-3.0, -4.0}, {-2.0, -5.0}, { 2.0, -5.0}, { 3.0, -4.0}, { 3.0, -1.0}, { 2.0,  0.0}, {-2.0,  1.0}, {-3.0,  2.0}, {-3.0,  5.0}, { 3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Numeral 3
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_NUMERAL, dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   13, {{-3.0, -4.0}, {-2.0, -5.0}, { 2.0, -5.0}, { 3.0, -4.0}, { 3.0, -1.0}, { 2.0,  0.0}, {-2.0,  0.0}, { 2.0,  0.0}, { 3.0,  1.0}, { 3.0,  4.0}, { 2.0,  5.0}, {-2.0,  5.0}, {-3.0,  4.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Numeral 4
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_NUMERAL, dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{-2.0, -5.0}, {-3.0,  0.0}, { 3.0,  0.0}, { 3.0, -5.0}, { 3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Numeral 5
	//{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_NUMERAL, dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   10, {{ 3.0, -5.0}, {-3.0, -5.0}, {-3.0,  0.0}, {-2.0, -1.0}, { 2.0, -1.0}, { 3.0,  0.0}, { 3.0,  4.0}, { 2.0,  5.0}, {-2.0,  5.0}, {-3.0,  4.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_NUMERAL, dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   11, {{ 3.0, -5.0}, {-2.5, -5.0}, {-2.8, -2.0}, { 0.0, -2.0}, { 2.0, -1.5}, { 3.0,  0.0}, { 3.0,  3.0}, { 2.0,  4.5}, { 0.0,  5.0}, {-1.0,  5.0}, {-3.0,  4.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Numeral 6
	//{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_NUMERAL, dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   12, {{ 3.0, -4.0}, { 2.0, -5.0}, {-2.0, -5.0}, {-3.0, -4.0}, {-3.0,  4.0}, {-2.0,  5.0}, { 2.0,  5.0}, { 3.0,  4.0}, { 3.0,  0.0}, { 2.0, -1.0}, {-2.0, -1.0}, {-3.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_NUMERAL, dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   12, {{ 3.0, -5.0}, { 1.0, -5.0}, {-1.0, -4.0}, {-2.0, -3.0}, {-3.0,  0.0}, {-3.0,  3.0}, {-2.0,  5.0}, { 2.0,  5.0}, { 3.0,  4.0}, { 3.0,  1.0}, { 2.0,  0.0}, {-3.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Numeral 7
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_NUMERAL, dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{-3.0, -5.0}, { 3.0, -5.0}, { 3.0, -3.0}, { 1.0, -1.0}, { 1.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Numeral 8
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_NUMERAL, dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   12, {{ 3.5, -4.0}, {-3.0,  2.0}, {-3.0,  4.0}, {-2.0,  5.0}, { 2.0,  5.0}, { 3.0,  4.0}, { 3.0,  2.0}, {-3.0, -3.0}, {-3.0, -4.0}, {-2.0, -5.0}, { 1.0, -5.0}, { 3.0, -3.5}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Numeral 9
	//{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_NUMERAL, dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   12, {{ 3.0,  0.0}, { 2.0,  1.0}, {-2.0,  1.0}, {-3.0,  0.0}, {-3.0, -4.0}, {-2.0, -5.0}, { 2.0, -5.0}, { 3.0, -4.0}, { 3.0,  4.0}, { 2.0,  5.0}, {-2.0,  5.0}, {-3.0,  4.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_NUMERAL, dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   12, {{-3.0,  5.0}, {-1.0,  5.0}, { 1.0,  4.0}, { 2.0,  3.0}, { 3.0,  0.0}, { 3.0, -3.0}, { 2.0, -5.0}, {-2.0, -5.0}, {-3.0, -4.0}, {-3.0, -1.0}, {-2.0,  0.0}, { 3.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter A
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{-3.0,  5.0}, { 0.0, -5.0}, { 3.0,  5.0}, { 1.9,  1.0}, {-2.0,  1.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter B
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   12, {{-3.0,  5.0}, {-3.0, -5.0}, { 2.0, -5.0}, { 3.0, -4.0}, { 3.0, -1.0}, { 2.0,  0.0}, {-3.0,  0.0}, { 2.0,  0.0}, { 3.0,  1.0}, { 3.0,  4.0}, { 2.0,  5.0}, {-3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter C
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   10, {{ 3.0, -3.0}, { 3.0, -4.0}, { 2.0, -5.0}, {-2.0, -5.0}, {-3.0, -4.0}, {-3.0,  4.0}, {-2.0,  5.0}, { 2.0,  5.0}, { 3.0,  4.0}, { 3.0,  2.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter D
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    7, {{-3.0,  5.0}, {-3.0, -5.0}, { 2.0, -5.0}, { 3.0, -4.0}, { 3.0,  4.0}, { 2.0,  5.0}, {-3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter E
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    7, {{ 3.0, -5.0}, {-3.0, -5.0}, {-3.0,  0.0}, { 2.0,  0.0}, {-3.0,  0.0}, {-3.0,  5.0}, { 3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter F
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    6, {{ 3.0, -5.0}, {-3.0, -5.0}, {-3.0,  0.0}, { 2.0,  0.0}, {-3.0,  0.0}, {-3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter G
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   11, {{ 3.0, -3.0}, { 3.0, -4.0}, { 2.0, -5.0}, {-2.0, -5.0}, {-3.0, -4.0}, {-3.0,  4.0}, {-2.0,  5.0}, { 2.0,  5.0}, { 3.0,  4.0}, { 3.0,  1.0}, { 1.0,  1.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter H
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    6, {{-3.0, -5.0}, {-3.0,  5.0}, {-3.0,  0.0}, { 3.0,  0.0}, { 3.0, -5.0}, { 3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter I
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    6, {{-1.0, -5.0}, { 1.0, -5.0}, { 0.0, -5.0}, { 0.0,  5.0}, {-1.0,  5.0}, { 1.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter J
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    6, {{-3.0,  2.0}, {-3.0,  4.0}, {-2.0,  5.0}, { 2.0,  5.0}, { 3.0,  4.0}, { 3.0, -5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter K
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    6, {{-3.0, -5.0}, {-3.0,  5.0}, {-3.0,  1.0}, { 3.0, -5.0}, {-1.7,  0.2}, { 3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter L
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    3, {{-3.0, -5.0}, {-3.0,  5.0}, { 3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter M
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{-3.0,  5.0}, {-3.0, -5.0}, { 0.0, -2.0}, { 3.0, -5.0}, { 3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter N
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    4, {{-3.0,  5.0}, {-3.0, -5.0}, { 3.0,  5.0}, { 3.0, -5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter O
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    9, {{ 2.0, -5.0}, {-2.0, -5.0}, {-3.0, -4.0}, {-3.0,  4.0}, {-2.0,  5.0}, { 2.0,  5.0}, { 3.0,  4.0}, { 3.0, -4.0}, { 2.0, -5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter P
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    7, {{-3.0,  5.0}, {-3.0, -5.0}, { 2.0, -5.0}, { 3.0, -4.0}, { 3.0, -1.0}, { 2.0,  0.0}, {-3.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter Q
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   12, {{ 3.0,  4.0}, { 2.0,  5.0}, {-2.0,  5.0}, {-3.0,  4.0}, {-3.0, -4.0}, {-2.0, -5.0}, { 2.0, -5.0}, { 3.0, -4.0}, { 3.0,  4.0}, { 2.5,  4.5}, { 0.0,  2.0}, { 3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter R
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    9, {{-3.0,  5.0}, {-3.0, -5.0}, { 2.0, -5.0}, { 3.0, -4.0}, { 3.0, -1.0}, { 2.0,  0.0}, {-3.0,  0.0}, { 1.0,  0.0}, { 3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter S
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,   14, {{ 3.0, -3.0}, { 3.0, -4.0}, { 2.0, -5.0}, {-2.0, -5.0}, {-3.0, -4.0}, {-3.0, -2.0}, {-2.0, -1.0}, { 2.0,  0.0}, { 3.0,  1.0}, { 3.0,  4.0}, { 2.0,  5.0}, {-2.0,  5.0}, {-3.0,  4.0}, {-3.0,  3.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter T
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    4, {{ 3.0, -5.0}, {-3.0, -5.0}, { 0.0, -5.0}, { 0.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter U
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    6, {{-3.0, -5.0}, {-3.0,  4.0}, {-2.0,  5.0}, { 2.0,  5.0}, { 3.0,  4.0}, { 3.0, -5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter V
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    3, {{-3.0, -5.0}, { 0.0,  5.0}, { 3.0, -5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter W
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{-3.0, -5.0}, {-2.0,  5.0}, { 0.0,  1.0}, { 2.0,  5.0}, { 3.0, -5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter X
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    6, {{-3.0, -5.0}, { 0.0,  0.0}, {-3.0,  5.0}, { 3.0, -5.0}, { 0.0,  0.0}, { 3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter Y
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{-3.0, -5.0}, { 0.0,  0.0}, { 3.0, -5.0}, { 0.0,  0.0}, { 0.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter Z
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    4, {{-3.0, -5.0}, { 3.0, -5.0}, {-3.0,  5.0}, { 3.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Letter small c					'c'
	{ShapeType_Symbol,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    8, {{ 2.0, -1.0}, { 1.0, -2.0}, {-1.0, -2.0}, {-2.0, -1.0}, {-2.0,  1.0}, {-1.0,  2.0}, { 1.0,  2.0}, { 2.0,  1.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Symbol Circle for (c)opyright	'o'
	{ShapeType_Symbol,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    9, {{ 0.0,  5.0}, { 3.5,  3.5}, { 5.0,  0.0}, { 3.5, -3.5}, { 0.0, -5.0}, {-3.5, -3.5}, {-5.0,  0.0}, {-3.5,  3.5}, { 0.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Symbol Left Parenthesis			'('
	{ShapeType_Symbol,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    4, {{ 1.0, -5.0}, {-1.0, -2.0}, {-1.0,  2.0}, { 1.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Symbol Right Parenthesis			')'
	{ShapeType_Symbol,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    4, {{-1.0, -5.0}, { 1.0, -2.0}, { 1.0,  2.0}, {-1.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Symbol Underscore				'_'
	{ShapeType_Symbol,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    2, {{-5.0,  7.0}, { 5.0,  7.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Symbol Cursor (box)				'&'
	{ShapeType_Symbol,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{-5.0,  7.0}, {-5.0, -7.0}, { 5.0, -7.0}, { 5.0,  7.0}, {-5.0,  7.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Symbol Up-Arrow					'^'
	{ShapeType_Symbol,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{-3.0,  0.0}, { 0.0, -5.0}, { 3.0,  0.0}, { 0.0, -5.0}, { 0.0,  3.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Symbol Down-Arrow				'v'
	{ShapeType_Symbol,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{-3.0,  0.0}, { 0.0,  5.0}, { 3.0,  0.0}, { 0.0,  5.0}, { 0.0, -3.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Symbol Left-Arrow				'<'
	{ShapeType_Symbol,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{ 0.0,  3.0}, {-5.0,  0.0}, { 0.0, -3.0}, {-5.0,  0.0}, { 3.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Symbol Right-Arrow				'>'
	{ShapeType_Symbol,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{ 0.0,  3.0}, { 5.0,  0.0}, { 0.0, -3.0}, { 5.0,  0.0}, {-3.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Symbol Square					'['
	{ShapeType_Symbol,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    5, {{-5.0,  5.0}, {-5.0, -5.0}, { 5.0, -5.0}, { 5.0,  5.0}, {-5.0,  5.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}},
	// Blank							' '
	{ShapeType_Text,	FALSE, FALSE,  AUTOROTATE_LETTER,  dptHITPOINT_NONE,    dptHITPOINT_NONE,    dptHITPOINT_NONE,    DISTANCEHIT_SHIP,    0, {{ 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}, { 0.0,  0.0}}}

	// When you add to this list, remember to adjust values in shapes.h, and potentially CSpriteList::CreateSprites().
};


// ****************************************
// * Functions
// ****************************************





// End of file.

