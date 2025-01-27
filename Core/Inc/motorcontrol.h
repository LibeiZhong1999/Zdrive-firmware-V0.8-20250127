#ifndef __MOTORCONTROL_H
#define __MOTORCONTROL_H

#include "main.h"
#include "stdbool.h"
#include "tim.h"
#include "arm_math.h"

struct tri_vector
{
	float fir;
	float sec;
	float thi;
	bool sta;
};

extern const float one_div_sq_three,sq_three,rad_to_deg,pulse_to_rad; 
extern const uint32_t ARR;
extern const float pi,cycle,one_div_cycle,Ld,Lq,R,pole_pair,angle_offset;
extern float sintest,costest,sintest1,costest1;
extern float ia,ib,ic,ua,ub,uc,tem,dc_bus,ia_carli,ib_carli,ic_carli,theta_e,scalefactor;
extern uint32_t ia_u32,ib_u32,ic_u32,ia_carli_u32,ib_carli_u32,ic_carli_u32;
extern float id_ref,id,id_pre,kp_d,ki_d,integ_d,ud,ud_mod;
extern float iq_ref,iq,iq_pre,kp_q,ki_q,integ_q,uq,uq_mod;
extern float theta_m,theta_m_pre,vel_m;
extern float vel_ref,kp_vel,ki_vel,integ_vel;
extern float pos_ref,kp_pos,integ_pos;
extern float vel_fil,vel_fil_pre;
extern	float del_pos;
extern int16_t encPositionA;
/*
*alpha beta is standized to (0,1)
**corresponding to the three phase voltage vector coordinate system standized by Vdc
*return value (tA,tB,tC,status) 
**tA is Duty cycle of axis A in [0,1],corresponding the time of high volatge
**status is bool type,identifies whether tA/tB/tC is effective
*/
struct tri_vector SVPWM(float alpha,float beta);

/*
*transfer 3 phase duty cycle volt_v-(tA,tB,tC,status) to PWM CCR
* tA/B/C is in [0,1],corresponding the time ratio of high volatge
*/
void apply_pwm(struct tri_vector volt_abc_v);

/*
	basing on ADC,get three phase current
*/
void get_cur(void);

/*
	*transfer current vector from abc coordinate system to dq coordinate system
	*cur_abc_v=(ia,ib,ic,status)   theta_tf is the angle at which the d-axis is ahead of the a-axis  !rad
	*return para=(id,iq,default,status)
	*After transformation, the vector modulus does not change
	*In the n-phase coordinate system, if each axis component is sinusoidal and the amplitude is A, 
	the phases are 2 pi/n rad apart from each other. 
	Then the composite vector modulus is nA/2 and the angle is equal to the sinusoidal phase of the prime phase component
	*so if current vector modulus is S,then the amplitude of axis component of abc is 2S/3, dq and alpha-beta is S
*/
struct tri_vector Clark_Park_Tf(struct tri_vector cur_abc_v,float theta_tf);

/*
	*transfer voltage vector from dq coordinate system to alpha-beta coordinate system
	*volt_dq_v=(vd,vq,default,status)   theta_tf is the angle at which the d-axis is ahead of the a-axis  !rad
	*return para=(v_alpha,v_beta,default,status)
	*After transformation, the vector modulus does not change
*/
struct tri_vector Inv_Park_Tf(struct tri_vector volt_dq_v,float theta_tf);

/*
	current loop
*/
void current_loop(void);


/*
	speed loop
*/
void speed_loop(void);

/*
	pos loop
*/
void pos_loop(void);

#endif