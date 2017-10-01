#include <ros/ros.h>
#include <iostream>
#include "keyboard.h"
#include <sensor_msgs/Joy.h>

uint16_t KEY_UNKNOWN=0;
uint16_t KEY_FIRST=0;
uint16_t KEY_BACKSPACE=8;
uint16_t KEY_TAB=9;
uint16_t KEY_CLEAR=12;
uint16_t KEY_RETURN=13;
uint16_t KEY_PAUSE=19;
uint16_t KEY_ESCAPE=27;
uint16_t KEY_SPACE=32;
uint16_t KEY_EXCLAIM=33;
uint16_t KEY_QUOTEDBL=34;
uint16_t KEY_HASH=35;
uint16_t KEY_DOLLAR=36;
uint16_t KEY_AMPERSAND=38;
uint16_t KEY_QUOTE=39;
uint16_t KEY_LEFTPAREN=40;
uint16_t KEY_RIGHTPAREN=41;
uint16_t KEY_ASTERISK=42;
uint16_t KEY_PLUS=43;
uint16_t KEY_COMMA=44;
uint16_t KEY_MINUS=45;
uint16_t KEY_PERIOD=46;
uint16_t KEY_SLASH=47;
uint16_t KEY_0=48;
uint16_t KEY_1=49;
uint16_t KEY_2=50;
uint16_t KEY_3=51;
uint16_t KEY_4=52;
uint16_t KEY_5=53;
uint16_t KEY_6=54;
uint16_t KEY_7=55;
uint16_t KEY_8=56;
uint16_t KEY_9=57;
uint16_t KEY_COLON=58;
uint16_t KEY_SEMICOLON=59;
uint16_t KEY_LESS=60;
uint16_t KEY_EQUALS=61;
uint16_t KEY_GREATER=62;
uint16_t KEY_QUESTION=63;
uint16_t KEY_AT=64;
uint16_t KEY_LEFTBRACKET=91;
uint16_t KEY_BACKSLASH=92;
uint16_t KEY_RIGHTBRACKET=93;
uint16_t KEY_CARET=94;
uint16_t KEY_UNDERSCORE=95;
uint16_t KEY_BACKQUOTE=96;
uint16_t KEY_a=97;
uint16_t KEY_b=98;
uint16_t KEY_c=99;
uint16_t KEY_d=100;
uint16_t KEY_e=101;
uint16_t KEY_f=102;
uint16_t KEY_g=103;
uint16_t KEY_h=104;
uint16_t KEY_i=105;
uint16_t KEY_j=106;
uint16_t KEY_k=107;
uint16_t KEY_l=108;
uint16_t KEY_m=109;
uint16_t KEY_n=110;
uint16_t KEY_o=111;
uint16_t KEY_p=112;
uint16_t KEY_q=113;
uint16_t KEY_r=114;
uint16_t KEY_s=115;
uint16_t KEY_t=116;
uint16_t KEY_u=117;
uint16_t KEY_v=118;
uint16_t KEY_w=119;
uint16_t KEY_x=120;
uint16_t KEY_y=121;
uint16_t KEY_z=122;
uint16_t KEY_DELETE=127;
uint16_t KEY_WORLD_0=160;
uint16_t KEY_WORLD_1=161;
uint16_t KEY_WORLD_2=162;
uint16_t KEY_WORLD_3=163;
uint16_t KEY_WORLD_4=164;
uint16_t KEY_WORLD_5=165;
uint16_t KEY_WORLD_6=166;
uint16_t KEY_WORLD_7=167;
uint16_t KEY_WORLD_8=168;
uint16_t KEY_WORLD_9=169;
uint16_t KEY_WORLD_10=170;
uint16_t KEY_WORLD_11=171;
uint16_t KEY_WORLD_12=172;
uint16_t KEY_WORLD_13=173;
uint16_t KEY_WORLD_14=174;
uint16_t KEY_WORLD_15=175;
uint16_t KEY_WORLD_16=176;
uint16_t KEY_WORLD_17=177;
uint16_t KEY_WORLD_18=178;
uint16_t KEY_WORLD_19=179;
uint16_t KEY_WORLD_20=180;
uint16_t KEY_WORLD_21=181;
uint16_t KEY_WORLD_22=182;
uint16_t KEY_WORLD_23=183;
uint16_t KEY_WORLD_24=184;
uint16_t KEY_WORLD_25=185;
uint16_t KEY_WORLD_26=186;
uint16_t KEY_WORLD_27=187;
uint16_t KEY_WORLD_28=188;
uint16_t KEY_WORLD_29=189;
uint16_t KEY_WORLD_30=190;
uint16_t KEY_WORLD_31=191;
uint16_t KEY_WORLD_32=192;
uint16_t KEY_WORLD_33=193;
uint16_t KEY_WORLD_34=194;
uint16_t KEY_WORLD_35=195;
uint16_t KEY_WORLD_36=196;
uint16_t KEY_WORLD_37=197;
uint16_t KEY_WORLD_38=198;
uint16_t KEY_WORLD_39=199;
uint16_t KEY_WORLD_40=200;
uint16_t KEY_WORLD_41=201;
uint16_t KEY_WORLD_42=202;
uint16_t KEY_WORLD_43=203;
uint16_t KEY_WORLD_44=204;
uint16_t KEY_WORLD_45=205;
uint16_t KEY_WORLD_46=206;
uint16_t KEY_WORLD_47=207;
uint16_t KEY_WORLD_48=208;
uint16_t KEY_WORLD_49=209;
uint16_t KEY_WORLD_50=210;
uint16_t KEY_WORLD_51=211;
uint16_t KEY_WORLD_52=212;
uint16_t KEY_WORLD_53=213;
uint16_t KEY_WORLD_54=214;
uint16_t KEY_WORLD_55=215;
uint16_t KEY_WORLD_56=216;
uint16_t KEY_WORLD_57=217;
uint16_t KEY_WORLD_58=218;
uint16_t KEY_WORLD_59=219;
uint16_t KEY_WORLD_60=220;
uint16_t KEY_WORLD_61=221;
uint16_t KEY_WORLD_62=222;
uint16_t KEY_WORLD_63=223;
uint16_t KEY_WORLD_64=224;
uint16_t KEY_WORLD_65=225;
uint16_t KEY_WORLD_66=226;
uint16_t KEY_WORLD_67=227;
uint16_t KEY_WORLD_68=228;
uint16_t KEY_WORLD_69=229;
uint16_t KEY_WORLD_70=230;
uint16_t KEY_WORLD_71=231;
uint16_t KEY_WORLD_72=232;
uint16_t KEY_WORLD_73=233;
uint16_t KEY_WORLD_74=234;
uint16_t KEY_WORLD_75=235;
uint16_t KEY_WORLD_76=236;
uint16_t KEY_WORLD_77=237;
uint16_t KEY_WORLD_78=238;
uint16_t KEY_WORLD_79=239;
uint16_t KEY_WORLD_80=240;
uint16_t KEY_WORLD_81=241;
uint16_t KEY_WORLD_82=242;
uint16_t KEY_WORLD_83=243;
uint16_t KEY_WORLD_84=244;
uint16_t KEY_WORLD_85=245;
uint16_t KEY_WORLD_86=246;
uint16_t KEY_WORLD_87=247;
uint16_t KEY_WORLD_88=248;
uint16_t KEY_WORLD_89=249;
uint16_t KEY_WORLD_90=250;
uint16_t KEY_WORLD_91=251;
uint16_t KEY_WORLD_92=252;
uint16_t KEY_WORLD_93=253;
uint16_t KEY_WORLD_94=254;
uint16_t KEY_WORLD_95=255;
uint16_t KEY_KP0=256;
uint16_t KEY_KP1=257;
uint16_t KEY_KP2=258;
uint16_t KEY_KP3=259;
uint16_t KEY_KP4=260;
uint16_t KEY_KP5=261;
uint16_t KEY_KP6=262;
uint16_t KEY_KP7=263;
uint16_t KEY_KP8=264;
uint16_t KEY_KP9=265;
uint16_t KEY_KP_PERIOD=266;
uint16_t KEY_KP_DIVIDE=267;
uint16_t KEY_KP_MULTIPLY=268;
uint16_t KEY_KP_MINUS=269;
uint16_t KEY_KP_PLUS=270;
uint16_t KEY_KP_ENTER=271;
uint16_t KEY_KP_EQUALS=272;
uint16_t KEY_UP=273;
uint16_t KEY_DOWN=274;
uint16_t KEY_RIGHT=275;
uint16_t KEY_LEFT=276;
uint16_t KEY_INSERT=277;
uint16_t KEY_HOME=278;
uint16_t KEY_END=279;
uint16_t KEY_PAGEUP=280;
uint16_t KEY_PAGEDOWN=281;
uint16_t KEY_F1=282;
uint16_t KEY_F2=283;
uint16_t KEY_F3=284;;
uint16_t KEY_F4=285;
uint16_t KEY_F5=286;
uint16_t KEY_F6=287;;
uint16_t KEY_F7=288;
uint16_t KEY_F8=289;
uint16_t KEY_F9=290;
uint16_t KEY_F10=291;
uint16_t KEY_F11=292;
uint16_t KEY_F12=293;
uint16_t KEY_F13=294;
uint16_t KEY_F14=295;
uint16_t KEY_F15=296;
uint16_t KEY_NUMLOCK=300;
uint16_t KEY_CAPSLOCK=301;
uint16_t KEY_SCROLLOCK=302;
uint16_t KEY_RSHIFT=303;
uint16_t KEY_LSHIFT=304;
uint16_t KEY_RCTRL=305;
uint16_t KEY_LCTRL=306;
uint16_t KEY_RALT=307;
uint16_t KEY_LALT=308;
uint16_t KEY_RMETA=309;
uint16_t KEY_LMETA=310;
uint16_t KEY_LSUPER=311;
uint16_t KEY_RSUPER=312;
uint16_t KEY_MODE=313;
uint16_t KEY_COMPOSE=314;
uint16_t KEY_HELP=315;
uint16_t KEY_PRINT=316;
uint16_t KEY_SYSREQ=317;
uint16_t KEY_BREAK=318;
uint16_t KEY_MENU=319;
uint16_t KEY_POWER=320;
uint16_t KEY_EURO=321;
uint16_t KEY_UNDO=322;
uint16_t MODIFIER_NONE=0;
uint16_t MODIFIER_LSHIFT=1;
uint16_t MODIFIER_RSHIFT=2;
uint16_t MODIFIER_LCTRL=64;
uint16_t MODIFIER_RCTRL=128;
uint16_t MODIFIER_LALT=256;
uint16_t MODIFIER_RALT=512;
uint16_t MODIFIER_LMETA=1024;
uint16_t MODIFIER_RMETA=2048;
uint16_t MODIFIER_NUM=4096;
uint16_t MODIFIER_CAPS=8192;
uint16_t MODIFIER_MODE=16384;
uint16_t MODIFIER_RESERVED=32768;

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "keyboard_to_joy");
  ros::NodeHandle n;

  ros::Publisher joy_pub = n.advertise<sensor_msgs::Joy>("/joy", 5);

  keyboard::Keyboard kbd(1, 5);
  
  ros::Rate r(50);
  
  
  bool pressed, new_event;
  uint16_t code, modifiers;
  
  while (ros::ok()) {
      
    sensor_msgs::Joy joy_msg;
    joy_msg.axes.assign(8, 0.0);
    joy_msg.buttons.assign(8, 0.0);
    
    if (kbd.get_key(new_event, pressed, code, modifiers) && new_event) {
        
        if(pressed){
            if(code == KEY_UP){
                joy_msg.axes[7] = 1.0;
            }
            if(code == KEY_DOWN){
                joy_msg.axes[7] = -1.0;
            }
            if(code == KEY_LEFT){
                joy_msg.axes[6] = 1.0;
            }
            if(code == KEY_RIGHT){
                joy_msg.axes[6] = -1.0;
            }
            if(code == KEY_w){
                joy_msg.axes[1] = 1.0;
            }
            if(code == KEY_s){
                joy_msg.axes[1] = -1.0;
            }
            if(code == KEY_t){
                joy_msg.axes[3] = 1.0;
            }
            if(code == KEY_g){
                joy_msg.axes[3] = -1.0;
            }
            if(code == KEY_a){
                joy_msg.axes[0] = 1.0;
            }
            if(code == KEY_d){
                joy_msg.axes[0] = -1.0;
            }
            if(code == KEY_z){
                joy_msg.axes[2] = 1.0;
            }
            if(code == KEY_x){
                joy_msg.axes[2] = -1.0;
            }
            if(code == KEY_TAB){
                joy_msg.buttons[7] = 1.0;
            }
        }
    }
    
    joy_pub.publish(joy_msg);
    
    ros::spinOnce();
    r.sleep();
  }
  
  ros::waitForShutdown();
}
