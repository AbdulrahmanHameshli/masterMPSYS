function RHS = RHS(in1,in2,in3)
%RHS
%    RHS = RHS(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    13-Feb-2023 11:09:47

dq1 = in2(1,:);
dq2 = in2(2,:);
q1 = in1(1,:);
q2 = in1(2,:);
tau1 = in3(1,:);
tau2 = in3(2,:);
t2 = cos(q1);
t3 = cos(q2);
t4 = sin(q1);
t5 = sin(q2);
t6 = q1+q2;
t7 = cos(t6);
t8 = sin(t6);
t9 = t2.*5.0e+1;
t10 = t4.*5.0e+1;
t11 = t2.*t3.*2.5e+1;
t12 = t2.*t5.*2.5e+1;
t13 = t3.*t4.*2.5e+1;
t14 = t4.*t5.*2.5e+1;
t15 = (t2.*t3)./2.0;
t16 = t2.*t3.*2.45e+2;
t17 = (t2.*t5)./2.0;
t18 = (t3.*t4)./2.0;
t19 = (t4.*t5)./2.0;
t21 = t4.*t5.*2.45e+2;
t20 = -t14;
t22 = -t16;
t23 = -t19;
t24 = t7+t8;
t26 = t12+t13;
t28 = t17+t18;
t25 = dq2.*t24;
t27 = t11+t20;
t29 = t15+t23;
t30 = t4+t28;
t32 = t10+t26;
t31 = t2+t29;
t33 = t9+t27;
t34 = t26.*t29;
t35 = t27.*t28;
t37 = t27.*t30;
t46 = t29.*t32;
t36 = t26.*t31;
t38 = -t34;
t39 = -t35;
t40 = t34./2.0;
t41 = t35./2.0;
t42 = -t37;
t44 = t37./2.0;
t45 = t28.*t33;
t48 = -t46;
t50 = t46./2.0;
t43 = t36./2.0;
t47 = -t44;
t49 = t45./2.0;
t51 = -t50;
t52 = t35+t36+t38+t42;
t53 = dq2.*(t34-t36+t37+t39).*(-1.0./2.0);
t54 = t34+t39+t45+t48;
t55 = t36+t42+t45+t48;
t56 = (dq1.*t55)./2.0;
RHS = [dq1.*-1.0e+1-t2.*7.35e+2+t21+t22+tau1+dq2.*(t53+t56+dq2.*(t40-t41+t49+t51)+dq1.*(t43+t47+t49+t51));dq2.*-1.0e+1+t21+t22+tau2+dq2.*((dq1.*t54)./2.0-dq1.*(t40-t41-t43+t44))-dq1.*(t56-(dq2.*(t34-t36+t37+t39))./2.0)-(dq1.*dq2.*t54)./2.0;dq1.*(t25+dq1.*(t2+t4+t24))+dq2.*(t25+dq1.*t24)];
