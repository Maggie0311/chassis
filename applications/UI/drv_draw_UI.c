#include "drv_draw_UI.h"
#include "drv_send_UI.h"
// 主要实现的功能为：
// 1 首先要实现注册不同类型的UI元素，实现以不同频次刷新
// 2 能够实现自动打包和拼包处理，实现字符串和图形元素分开处理

//该文件主要实现的功能是：实现上层的注册和规划需要发送的UI元素

//		有一个数组，用于记录本次循环中需要发送的元素所在的位置（下标），在元素集合数组中查找该元素即可
//      设计两个数组集合，用于分开放置图形元素和字符元素，后续查找和发送也是用两个，先图形，后数组

//		需要注册回调函数，回调函数的主要作用是用来设置对应的元素的值，修改对应元素的类型等等（即图形相关配置或者字符内容）
//		循环需要找到对应需要发送的元素，同时调用其回调函数，将元素位置放入到对应的记录数组中，同时记录图形和字符的内容个数，这样在后边使用这两个值来规划发送数据包


void UI_Set_Line(UI_Object* object, UI_Object_Color_e color,int width, int layer, int x1, int y1, int x2, int y2)
{
    //object->detailDword1.operation = UI_operation;
    object->detailDword1.color = color;
    object->detailDword1.layer = layer;
    object->detailDword1.type = UiLine;
    object->detailDword2.width = width;
    object->detailDword2.x = x1;
    object->detailDword2.y = y1;
    object->detailDword3.line.x2 = x2;
    object->detailDword3.line.y2 = y2;
}

void UI_Set_Rect(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x1, int y1, int x2, int y2)
{
    //object->detailDword1.operation = UI_operation;
    object->detailDword1.color = color;
    object->detailDword1.layer = layer;
    object->detailDword1.type = UiRect;
    object->detailDword2.width = width;
    object->detailDword2.x = x1;
    object->detailDword2.y = y1;
    object->detailDword3.line.x2 = x2;
    object->detailDword3.line.y2 = y2;
}

void UI_Set_Circle(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x, int y, int radius)
{
    //object->detailDword1.operation = UI_operation;
    object->detailDword1.color = color;
    object->detailDword1.layer = layer;
    object->detailDword1.type = UiCircle;
    object->detailDword2.width = width;
    object->detailDword2.x = x;
    object->detailDword2.y = y;
    object->detailDword3.circle.radius = radius;

}

void UI_Set_Ellipse(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x, int y, int xSemiaxis, int ySemiaxis)
{
    //object->detailDword1.operation = UI_operation;
    object->detailDword1.color = color;
    object->detailDword1.layer = layer;
    object->detailDword1.type = UiEllipse;
    object->detailDword2.width = width;
    object->detailDword2.x = x;
    object->detailDword2.y = y;
    object->detailDword3.ellipse.xSemiaxis = xSemiaxis;
    object->detailDword3.ellipse.ySemiaxis = ySemiaxis;

}

void UI_Set_Arc(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x, int y, int xSemiaxis, int ySemiaxis, int startAngle, int endAngle)
{
    //object->detailDword1.operation = UI_operation;
    object->detailDword1.color = color;
    object->detailDword1.layer = layer;
    object->detailDword1.type = UiArc;
    object->detailDword2.width = width;
    object->detailDword2.x = x;
    object->detailDword2.y = y;
    object->detailDword3.ellipse.xSemiaxis = xSemiaxis;
    object->detailDword3.ellipse.ySemiaxis = ySemiaxis;
    object->detailDword1.detailA = startAngle;
    object->detailDword1.detailB = endAngle;
}

void UI_Set_Float(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x, int y, int fontSize, float value)
{
    //object->detailDword1.operation = UI_operation;
    object->detailDword1.color = color;
    object->detailDword1.layer = layer;
    object->detailDword1.type = UiFloat;
    object->detailDword2.width = width;
    object->detailDword2.x = x;
    object->detailDword2.y = y;
    object->detailDword1.detailA = fontSize;
    object->detailDword3.floatVal = (int)(value*1000);
}

void UI_Set_Int(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x, int y, int fontSize, int value)
{
    //object->detailDword1.operation = UI_operation;
    object->detailDword1.color = color;
    object->detailDword1.layer = layer;
    object->detailDword1.type = UiInt;
    object->detailDword2.width = width;
    object->detailDword2.x = x;
    object->detailDword2.y = y;
    object->detailDword1.detailA = fontSize;
    object->detailDword3.intVal = value;
}

void UI_Set_String(UI_Object* object, UI_Object_Color_e color, int width, int layer, int x, int y, int fontSize, char* str_p,unsigned char str_len)
{
    //object->detailDword1.operation = UI_operation;
    object->detailDword1.color = color;
    object->detailDword1.layer = layer;
    object->detailDword1.type = UiStr;
    object->detailDword2.width = width;
    object->detailDword2.x = x;
    object->detailDword2.y = y;
    object->detailDword1.detailA = fontSize;
    object->detailDword1.detailB= str_len;          //设置字符长度
    object->detailDword3.strVal = str_p;
}

void UI_Set_Color(UI_Object* object, UI_Object_Color_e color)
{
    object->detailDword1.color = color;
}

void UI_Set_Width(UI_Object* object, int width)
{
    object->detailDword2.width = width;
}

/// @brief 检查UI对象的值是否发生变化，并根据变化情况调用回调函数进行处理，常用于那些需要触发式发送的UI元素（某次需要显示，某次需要删除）
/// @param now_value 现在的值
/// @param last_value 上次的值
/// @param value_when_display 显示时的值
/// @param fun 回调函数
void UI_Checkout_Object_Change(int now_value,int last_value,int value_when_display,CallBack_fun_p fun)
{
    if(fun== NULL) //如果回调函数为空，直接返回
    {
        return;
    }
    if(now_value != last_value) //如果当前值和上次值不相等，则需要刷新UI元素，包括创建和删除
    {
        if (now_value == value_when_display) //如果当前值和需要显示时候的值相等，则需要显示该元素
        {
            if(Drv_Get_Object_Exist(fun)==0) //如果该元素没有显示
            {
                Drv_Set_Object_Visualable(UI_CREATE, fun); //设置为创建操作
            }
            else 
                return ;    //如果该元素已经显示，则不需要再创建
        }
        else if (now_value != value_when_display) //如果当前值和显示的值不相等，则需要删除该元素
        {
            if(Drv_Get_Object_Exist(fun)==1) //如果该元素已经显示
            {
                Drv_Set_Object_Visualable(UI_DELDTE, fun); //设置为删除操作
            }
            else 
                return ;    //如果该元素没有显示，则不需要再删除
        }
    }
}

/// <summary>
///             UI复位逻辑是，发送命令删除所有图层，设置UI运行时间为0（所有注册过的UI元素，设置了运行周期和次数（次数还未归零）的元素都会在下次运行周期中至少运行一次
/// </summary>
void UI_Reset_All()
{
    Drv_Set_run_time(0);                            //设置运行次数为0，保证下一次注册的该显示出来的元素全部显示
    Clear_Layer(UI_Layer_Delete_ALL, Ui_layer_0);   //全部删除无需考虑哪个图层
    Drv_Set_Visualable_All();                       //设置全部元素的操作类型为创建操作
}

