#pragma once
#include "drv_send_UI.h"

void UI_Set_Operation(UI_Operation_e operation);
void UI_Set_Line(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x1, int y1, int x2, int y2);
void UI_Set_Rect(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x1, int y1, int x2, int y2);
void UI_Set_Circle(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x, int y, int radius);
void UI_Set_Ellipse(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x, int y, int xSemiaxis, int ySemiaxis);
void UI_Set_Arc(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x, int y, int xSemiaxis, int ySemiaxis, int startAngle, int endAngle);
void UI_Set_Float(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x, int y, int fontSize, float value);
void UI_Set_Int(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x, int y, int fontSize, int value);
void UI_Set_String(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x, int y, int fontSize, char* str_p,unsigned char str_len);
void UI_Set_Color(UI_Object* object, UI_Object_Color_e color);
void UI_Set_Width(UI_Object* object, int width);

void UI_Checkout_Object_Change(int now_value,int last_value,int value_when_display,CallBack_fun_p fun);

void UI_Reset_All();














