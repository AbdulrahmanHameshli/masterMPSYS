function RHS_withoutConstraint = RHS_withoutConstraint(in1,in2,in3)
%RHS_withoutConstraint
%    RHS_withoutConstraint = RHS_withoutConstraint(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    13-Feb-2023 11:25:46

dq1 = in2(1,:);
dq2 = in2(2,:);
dq3 = in2(3,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
tau1 = in3(1,:);
tau2 = in3(2,:);
tau3 = in3(3,:);
t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = sin(q1);
t6 = sin(q2);
t7 = sin(q3);
t8 = t2.*t3;
t9 = t2.*t6;
t10 = t3.*t5;
t11 = t5.*t6;
t12 = t2.*5.0e+1;
t13 = t5.*5.0e+1;
t14 = t8.*2.5e+1;
t15 = t8.*5.0e+1;
t16 = t9.*2.5e+1;
t17 = t10.*2.5e+1;
t18 = t9.*5.0e+1;
t19 = t10.*5.0e+1;
t20 = -t11;
t21 = t11.*2.5e+1;
t22 = t11.*5.0e+1;
t23 = t8./2.0;
t24 = t9./2.0;
t25 = t10./2.0;
t26 = t11./2.0;
t29 = t8.*7.35e+2;
t31 = t11.*7.35e+2;
t33 = t9+t10;
t27 = -t21;
t28 = -t22;
t30 = -t26;
t32 = -t29;
t34 = t8+t20;
t35 = t16+t17;
t37 = t4.*t33.*2.5e+1;
t38 = t7.*t33.*2.5e+1;
t39 = t24+t25;
t40 = (t4.*t33)./2.0;
t41 = (t7.*t33)./2.0;
t43 = t7.*t33.*2.45e+2;
t36 = t14+t27;
t42 = -t38;
t44 = t23+t30;
t45 = t4.*t34.*2.5e+1;
t46 = -t41;
t47 = t7.*t34.*2.5e+1;
t48 = t5+t39;
t49 = (t4.*t34)./2.0;
t50 = t4.*t34.*2.45e+2;
t51 = (t7.*t34)./2.0;
t54 = t13+t35;
t52 = t2+t44;
t53 = -t50;
t55 = t12+t36;
t56 = t35.*t44;
t57 = t36.*t39;
t58 = t37+t47;
t60 = t36.*t48;
t63 = t42+t45;
t66 = t40+t51;
t68 = t46+t49;
t72 = t44.*t54;
t59 = t35.*t52;
t61 = -t56;
t62 = -t57;
t64 = t56./2.0;
t65 = t57./2.0;
t67 = -t60;
t70 = t60./2.0;
t71 = t39.*t55;
t74 = -t72;
t76 = t72./2.0;
t78 = t33+t66;
t80 = t18+t19+t58;
t81 = t34+t68;
t82 = t15+t28+t63;
t86 = -t66.*(t38-t45);
t87 = -t58.*(t41-t49);
t88 = t58.*(t41-t49);
t89 = t66.*(t38-t45).*(-1.0./2.0);
t69 = t59./2.0;
t73 = -t70;
t75 = t71./2.0;
t77 = -t76;
t79 = t5+t78;
t83 = t2+t81;
t84 = t13+t80;
t85 = t12+t82;
t90 = t88.*(-1.0./2.0);
t91 = t88./2.0;
t92 = -t78.*(t38-t45);
t94 = t58.*t81;
t95 = t78.*(t38-t45);
t100 = -t80.*(t41-t49);
t103 = t66.*t82;
t106 = t80.*(t41-t49);
t120 = t78.*t82;
t121 = t80.*t81;
t93 = -t79.*(t38-t45);
t96 = t95.*(-1.0./2.0);
t97 = t58.*t83;
t98 = t79.*(t38-t45);
t99 = -t94;
t102 = t94./2.0;
t108 = -t103;
t109 = t106.*(-1.0./2.0);
t110 = t103./2.0;
t111 = t106./2.0;
t113 = -t84.*(t41-t49);
t114 = t66.*t85;
t115 = t84.*(t41-t49);
t122 = t79.*t82;
t123 = -t120;
t124 = t80.*t83;
t125 = -t121;
t126 = t120./2.0;
t128 = t121./2.0;
t130 = t78.*t85;
t133 = t81.*t84;
t138 = t86+t88+t94+t95;
t144 = dq2.*(t87+t103+t106+t66.*(t38-t45)).*(-1.0./2.0);
t151 = dq2.*(t94+t95+t103+t106).*(-1.0./2.0);
t101 = t98.*(-1.0./2.0);
t104 = t98./2.0;
t105 = -t102;
t107 = t97./2.0;
t112 = -t110;
t116 = -t114;
t117 = t115.*(-1.0./2.0);
t118 = t114./2.0;
t127 = -t122;
t129 = t122./2.0;
t131 = t124./2.0;
t134 = t130./2.0;
t135 = -t133;
t136 = t133./2.0;
t139 = t86+t88+t97+t98;
t140 = (dq3.*t138)./2.0;
t142 = t86+t88+t100+t108;
t146 = dq1.*(t87+t114+t115+t66.*(t38-t45)).*(-1.0./2.0);
t147 = t92+t97+t98+t99;
t148 = t92+t99+t100+t108;
t149 = dq3.*(t93+t94+t95-t97).*(-1.0./2.0);
t150 = t97+t98+t103+t106;
t154 = t97+t98+t114+t115;
t155 = dq1.*(t94+t95+t114+t115).*(-1.0./2.0);
t159 = dq2.*(t56-t59+t60+t62+t121+t122+t123-t124).*(-1.0./2.0);
t119 = -t118;
t132 = -t129;
t137 = -t136;
t141 = -t140;
t143 = (dq3.*t139)./2.0;
t145 = t86+t88+t113+t116;
t152 = (dq2.*t150)./2.0;
t153 = t92+t99+t113+t116;
t156 = t103+t106+t113+t116;
t157 = (dq1.*t154)./2.0;
t158 = t57+t59+t61+t67+t120+t124+t125+t127;
t160 = t56+t62+t71+t74+t121+t123+t130+t135;
t161 = t59+t67+t71+t74+t124+t127+t130+t135;
t162 = (dq1.*t161)./2.0;
mt1 = [dq1.*-1.0e+1-t2.*1.225e+3+t31+t32+t43+t53+tau1+dq2.*(t162-(dq3.*(t93+t94+t95-t97))./2.0-(dq2.*(t56-t59+t60+t62+t121+t122+t123-t124))./2.0+dq2.*(t64-t65+t75+t77-t126+t128+t134+t137)+dq1.*(t69+t73+t75+t77+t131+t132+t134+t137)-dq3.*(t110+t111+t117+t119))+dq3.*(t143+t152+t157+dq1.*(t104+t107+t115./2.0+t118)+dq3.*(t90+t115./2.0+t118+(t66.*(t38-t45))./2.0)+dq2.*(t95./2.0+t102+t115./2.0+t118))];
mt2 = [dq2.*-1.0e+1+t31+t32+t43+t53+tau2+dq2.*((dq1.*t160)./2.0-dq1.*(t64-t65-t69+t70-t126+t128+t129-t131))+dq3.*(t140+dq2.*(t95./2.0+t102+t110+t111)+(dq2.*(t94+t95+t103+t106))./2.0+(dq1.*(t94+t95+t114+t115))./2.0+dq1.*(t104+t107+t110+t111)+dq3.*(t90+t110+t111+(t66.*(t38-t45))./2.0))+dq1.*(-t162+(dq3.*(t93+t94+t95-t97))./2.0+(dq2.*(t56-t59+t60+t62+t121+t122+t123-t124))./2.0)+(dq1.*dq3.*t156)./2.0-(dq1.*dq2.*t160)./2.0];
mt3 = [dq3.*-1.0e+1+t43+t53+tau3-dq2.*((dq1.*t156)./2.0+dq1.*(t95./2.0+t101+t102-t107))-dq3.*((dq2.*(t87+t103+t106+t66.*(t38-t45)))./2.0+(dq1.*(t87+t114+t115+t66.*(t38-t45)))./2.0)+dq3.*(dq2.*(t89+t91+t95./2.0+t102)+dq1.*(t89+t91+t104+t107)+(dq2.*(t87+t103+t106+t66.*(t38-t45)))./2.0+(dq1.*(t87+t114+t115+t66.*(t38-t45)))./2.0)-dq2.*(t140+(dq2.*(t94+t95+t103+t106))./2.0+(dq1.*(t94+t95+t114+t115))./2.0)-dq1.*(t143+t152+t157)];
RHS_withoutConstraint = [mt1;mt2;mt3];