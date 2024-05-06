function [vel_hip_l_stance,vel_hip_r_stance] = hip_velocities(l1,l2,phi,phid,phi_lh,phi_rh,phi_lhd,phi_rhd,psid,psi_lh,psi_rh,psi,psi_lhd,psi_rhd,theta,thetad,theta_lh,theta_lk,theta_rh,theta_rk,theta_lhd,theta_lkd,theta_rhd,theta_rkd,w)
%HIP_VELOCITIES
%    [VEL_HIP_L_STANCE,VEL_HIP_R_STANCE] = HIP_VELOCITIES(L1,L2,PHI,PHID,PHI_LH,PHI_RH,PHI_LHD,PHI_RHD,PSID,PSI_LH,PSI_RH,PSI,PSI_LHD,PSI_RHD,THETA,THETAD,THETA_LH,THETA_LK,THETA_RH,THETA_RK,THETA_LHD,THETA_LKD,THETA_RHD,THETA_RKD,W)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    2024-05-05 11:00:18

t2 = cos(phi);
t3 = cos(phi_lh);
t4 = cos(phi_rh);
t5 = cos(psi_lh);
t6 = cos(psi_rh);
t7 = cos(psi);
t8 = cos(theta);
t9 = cos(theta_lh);
t10 = cos(theta_lk);
t11 = cos(theta_rh);
t12 = cos(theta_rk);
t13 = sin(phi);
t14 = sin(phi_lh);
t15 = sin(phi_rh);
t16 = sin(psi_lh);
t17 = sin(psi_rh);
t18 = sin(psi);
t19 = sin(theta);
t20 = sin(theta_lh);
t21 = sin(theta_lk);
t22 = sin(theta_rh);
t23 = sin(theta_rk);
t24 = t2.*t7.*w;
t25 = t7.*t8.*w;
t26 = t2.*t18.*w;
t27 = t7.*t13.*w;
t28 = t13.*t18.*w;
t29 = t18.*t19.*w;
t36 = l1.*t2.*t5.*t7.*t20;
t37 = l1.*t2.*t6.*t7.*t22;
t38 = l1.*t5.*t7.*t13.*t20;
t39 = l1.*t6.*t7.*t13.*t22;
t40 = l1.*t5.*t8.*t18.*t20;
t41 = l1.*t7.*t8.*t16.*t20;
t42 = l1.*t6.*t8.*t18.*t22;
t43 = l1.*t7.*t8.*t17.*t22;
t44 = l2.*t3.*t9.*t19.*t21;
t45 = l2.*t3.*t10.*t19.*t20;
t46 = l2.*t4.*t11.*t19.*t23;
t47 = l2.*t4.*t12.*t19.*t22;
t48 = l1.*t2.*t16.*t18.*t20;
t49 = l1.*t2.*t17.*t18.*t22;
t50 = l1.*t13.*t16.*t18.*t20;
t51 = l1.*t13.*t17.*t18.*t22;
t52 = l1.*t5.*t7.*t8.*t9.*t14;
t53 = l2.*t2.*t5.*t7.*t9.*t21;
t54 = l2.*t2.*t5.*t7.*t10.*t20;
t55 = l2.*t2.*t5.*t9.*t10.*t18;
t56 = l2.*t2.*t7.*t9.*t10.*t16;
t57 = l1.*t6.*t7.*t8.*t11.*t15;
t58 = l2.*t2.*t6.*t7.*t11.*t23;
t59 = l2.*t2.*t6.*t7.*t12.*t22;
t60 = l2.*t2.*t6.*t11.*t12.*t18;
t61 = l2.*t2.*t7.*t11.*t12.*t17;
t62 = l2.*t2.*t3.*t8.*t9.*t21;
t63 = l2.*t2.*t3.*t8.*t10.*t20;
t64 = l2.*t2.*t4.*t8.*t11.*t23;
t65 = l2.*t2.*t4.*t8.*t12.*t22;
t66 = l1.*t2.*t5.*t9.*t14.*t18;
t67 = l1.*t2.*t7.*t9.*t14.*t16;
t68 = l1.*t2.*t6.*t11.*t15.*t18;
t69 = l1.*t2.*t7.*t11.*t15.*t17;
t70 = l2.*t5.*t7.*t9.*t13.*t21;
t71 = l2.*t5.*t7.*t10.*t13.*t20;
t72 = l2.*t5.*t9.*t10.*t13.*t18;
t73 = l2.*t7.*t9.*t10.*t13.*t16;
t74 = l2.*t6.*t7.*t11.*t13.*t23;
t75 = l2.*t6.*t7.*t12.*t13.*t22;
t76 = l2.*t6.*t11.*t12.*t13.*t18;
t77 = l2.*t7.*t11.*t12.*t13.*t17;
t78 = l2.*t3.*t8.*t9.*t13.*t21;
t79 = l2.*t3.*t8.*t10.*t13.*t20;
t80 = l2.*t4.*t8.*t11.*t13.*t23;
t81 = l2.*t4.*t8.*t12.*t13.*t22;
t82 = l2.*t5.*t7.*t8.*t20.*t21;
t83 = l2.*t5.*t8.*t9.*t18.*t21;
t84 = l2.*t5.*t8.*t10.*t18.*t20;
t85 = l2.*t7.*t8.*t9.*t16.*t21;
t86 = l2.*t7.*t8.*t10.*t16.*t20;
t87 = l2.*t8.*t9.*t10.*t16.*t18;
t88 = l2.*t6.*t7.*t8.*t22.*t23;
t89 = l2.*t6.*t8.*t11.*t18.*t23;
t90 = l2.*t6.*t8.*t12.*t18.*t22;
t91 = l2.*t7.*t8.*t11.*t17.*t23;
t92 = l2.*t7.*t8.*t12.*t17.*t22;
t93 = l2.*t8.*t11.*t12.*t17.*t18;
t94 = l1.*t5.*t9.*t13.*t14.*t18;
t95 = l1.*t7.*t9.*t13.*t14.*t16;
t96 = l1.*t6.*t11.*t13.*t15.*t18;
t97 = l1.*t7.*t11.*t13.*t15.*t17;
t98 = l1.*t2.*t5.*t18.*t19.*t20;
t99 = l1.*t2.*t7.*t16.*t19.*t20;
t100 = l1.*t2.*t6.*t18.*t19.*t22;
t101 = l1.*t2.*t7.*t17.*t19.*t22;
t102 = l1.*t8.*t9.*t14.*t16.*t18;
t103 = l2.*t2.*t5.*t18.*t20.*t21;
t104 = l2.*t2.*t7.*t16.*t20.*t21;
t105 = l2.*t2.*t9.*t16.*t18.*t21;
t106 = l2.*t2.*t10.*t16.*t18.*t20;
t107 = l1.*t8.*t11.*t15.*t17.*t18;
t108 = l2.*t2.*t6.*t18.*t22.*t23;
t109 = l2.*t2.*t7.*t17.*t22.*t23;
t110 = l2.*t2.*t11.*t17.*t18.*t23;
t111 = l2.*t2.*t12.*t17.*t18.*t22;
t113 = l1.*t5.*t13.*t18.*t19.*t20;
t114 = l1.*t7.*t13.*t16.*t19.*t20;
t115 = l1.*t6.*t13.*t18.*t19.*t22;
t116 = l1.*t7.*t13.*t17.*t19.*t22;
t117 = l2.*t5.*t13.*t18.*t20.*t21;
t118 = l2.*t7.*t13.*t16.*t20.*t21;
t119 = l2.*t9.*t13.*t16.*t18.*t21;
t120 = l2.*t10.*t13.*t16.*t18.*t20;
t121 = l2.*t6.*t13.*t18.*t22.*t23;
t122 = l2.*t7.*t13.*t17.*t22.*t23;
t123 = l2.*t11.*t13.*t17.*t18.*t23;
t124 = l2.*t12.*t13.*t17.*t18.*t22;
t125 = l2.*t8.*t16.*t18.*t20.*t21;
t126 = l2.*t8.*t17.*t18.*t22.*t23;
t129 = l2.*t5.*t7.*t8.*t9.*t10;
t130 = l2.*t6.*t7.*t8.*t11.*t12;
t134 = l1.*t2.*t9.*t14.*t16.*t18.*t19;
t135 = l1.*t2.*t11.*t15.*t17.*t18.*t19;
t146 = l2.*t5.*t7.*t13.*t19.*t20.*t21;
t147 = l2.*t5.*t9.*t13.*t18.*t19.*t21;
t148 = l2.*t5.*t10.*t13.*t18.*t19.*t20;
t149 = l2.*t7.*t9.*t13.*t16.*t19.*t21;
t150 = l2.*t7.*t10.*t13.*t16.*t19.*t20;
t151 = l2.*t9.*t10.*t13.*t16.*t18.*t19;
t152 = l2.*t6.*t7.*t13.*t19.*t22.*t23;
t153 = l2.*t6.*t11.*t13.*t18.*t19.*t23;
t154 = l2.*t6.*t12.*t13.*t18.*t19.*t22;
t155 = l2.*t7.*t11.*t13.*t17.*t19.*t23;
t156 = l2.*t7.*t12.*t13.*t17.*t19.*t22;
t157 = l2.*t11.*t12.*t13.*t17.*t18.*t19;
t162 = l1.*t9.*t13.*t14.*t16.*t18.*t19;
t163 = l1.*t11.*t13.*t15.*t17.*t18.*t19;
t181 = l2.*t2.*t16.*t18.*t19.*t20.*t21;
t182 = l2.*t2.*t17.*t18.*t19.*t22.*t23;
t193 = l2.*t13.*t16.*t18.*t19.*t20.*t21;
t194 = l2.*t13.*t17.*t18.*t19.*t22.*t23;
t200 = l2.*t2.*t5.*t7.*t9.*t10.*t19;
t201 = l2.*t2.*t6.*t7.*t11.*t12.*t19;
t204 = l1.*t2.*t5.*t7.*t9.*t14.*t19;
t205 = l1.*t2.*t6.*t7.*t11.*t15.*t19;
t214 = l2.*t5.*t7.*t9.*t10.*t13.*t19;
t215 = l2.*t6.*t7.*t11.*t12.*t13.*t19;
t216 = l1.*t5.*t7.*t9.*t13.*t14.*t19;
t217 = l1.*t6.*t7.*t11.*t13.*t15.*t19;
t226 = l2.*t2.*t5.*t7.*t19.*t20.*t21;
t227 = l2.*t2.*t5.*t9.*t18.*t19.*t21;
t228 = l2.*t2.*t5.*t10.*t18.*t19.*t20;
t229 = l2.*t2.*t7.*t9.*t16.*t19.*t21;
t230 = l2.*t2.*t7.*t10.*t16.*t19.*t20;
t231 = l2.*t2.*t9.*t10.*t16.*t18.*t19;
t232 = l2.*t2.*t6.*t7.*t19.*t22.*t23;
t233 = l2.*t2.*t6.*t11.*t18.*t19.*t23;
t234 = l2.*t2.*t6.*t12.*t18.*t19.*t22;
t235 = l2.*t2.*t7.*t11.*t17.*t19.*t23;
t236 = l2.*t2.*t7.*t12.*t17.*t19.*t22;
t237 = l2.*t2.*t11.*t12.*t17.*t18.*t19;
t30 = t19.*t24;
t31 = t8.*t26;
t32 = t19.*t26;
t33 = t19.*t27;
t34 = t8.*t28;
t35 = t19.*t28;
t112 = -t36;
t127 = -t42;
t128 = -t50;
t131 = -t53;
t132 = -t54;
t133 = -t61;
t136 = t14.*t103;
t137 = t14.*t104;
t138 = t14.*t105;
t139 = t14.*t106;
t140 = t15.*t108;
t141 = t15.*t109;
t142 = t15.*t110;
t143 = t15.*t111;
t144 = -t62;
t145 = -t63;
t158 = -t69;
t159 = -t72;
t160 = -t73;
t161 = -t77;
t164 = t14.*t117;
t165 = t14.*t118;
t166 = t14.*t119;
t167 = t14.*t120;
t168 = t15.*t121;
t169 = t15.*t122;
t170 = t15.*t123;
t171 = t15.*t124;
t172 = -t78;
t173 = -t79;
t174 = -t80;
t175 = -t81;
t176 = -t82;
t177 = -t87;
t178 = -t88;
t179 = -t89;
t180 = -t90;
t183 = t14.*t125;
t184 = t15.*t126;
t185 = -t94;
t186 = -t95;
t187 = -t97;
t188 = -t101;
t189 = -t102;
t190 = -t103;
t191 = -t104;
t192 = -t108;
t195 = -t115;
t196 = -t119;
t197 = -t120;
t198 = -t121;
t199 = -t126;
t202 = t14.*t129;
t203 = t15.*t130;
t208 = t14.*t55;
t209 = t14.*t56;
t210 = t15.*t58;
t211 = t15.*t59;
t212 = t15.*t60;
t218 = t14.*t70;
t219 = t14.*t71;
t220 = t14.*t72;
t221 = t14.*t73;
t222 = t15.*t74;
t223 = t15.*t75;
t224 = t15.*t76;
t239 = t14.*t83;
t240 = t14.*t84;
t241 = t14.*t85;
t242 = t14.*t86;
t247 = t15.*t91;
t248 = t15.*t92;
t249 = t15.*t93;
t250 = t14.*t181;
t251 = t15.*t182;
t252 = -t134;
t253 = -t135;
t257 = -t146;
t258 = -t151;
t259 = -t152;
t260 = -t153;
t261 = -t154;
t262 = t14.*t193;
t263 = t15.*t194;
t264 = -t162;
t269 = -t194;
t270 = t14.*t200;
t271 = t15.*t201;
t272 = t14.*t214;
t273 = t15.*t215;
t274 = -t201;
t275 = t14.*t226;
t276 = t14.*t227;
t277 = t14.*t228;
t278 = t14.*t229;
t279 = t14.*t230;
t280 = t14.*t231;
t281 = t15.*t232;
t282 = t15.*t233;
t283 = t15.*t234;
t284 = t15.*t235;
t285 = t15.*t236;
t286 = t15.*t237;
t287 = -t205;
t292 = t14.*t147;
t293 = t14.*t148;
t294 = t14.*t149;
t295 = t14.*t150;
t300 = t15.*t155;
t301 = t15.*t156;
t302 = t15.*t157;
t306 = -t226;
t307 = -t231;
t308 = -t235;
t309 = -t236;
t310 = -t237;
t254 = -t136;
t255 = -t137;
t256 = -t140;
t265 = -t166;
t266 = -t167;
t267 = -t168;
t268 = -t184;
t288 = t14.*t131;
t289 = t14.*t132;
t290 = t15.*t133;
t303 = t14.*t159;
t304 = t14.*t160;
t305 = t15.*t161;
t311 = t14.*t176;
t312 = t14.*t177;
t313 = t15.*t178;
t314 = t15.*t179;
t315 = t15.*t180;
t316 = -t263;
t317 = -t271;
t318 = -t275;
t319 = -t280;
t320 = -t284;
t321 = -t285;
t322 = -t286;
t323 = t14.*t257;
t324 = t14.*t258;
et1 = psid.*(t25+t40+t41+t52+t83+t84+t85+t86+t183+t189+t202+t311+t312)+psi_lhd.*(t40+t41+t52+t83+t84+t85+t86+t183+t189+t202+t311+t312)-thetad.*(t29-l1.*t3.*t8.*t9-l2.*t3.*t8.*t9.*t10-l1.*t5.*t7.*t19.*t20+l2.*t3.*t8.*t20.*t21+l1.*t16.*t18.*t19.*t20-l2.*t5.*t7.*t9.*t19.*t21-l2.*t5.*t7.*t10.*t19.*t20+l1.*t5.*t9.*t14.*t18.*t19+l1.*t7.*t9.*t14.*t16.*t19+l2.*t9.*t16.*t18.*t19.*t21+l2.*t10.*t16.*t18.*t19.*t20+l2.*t5.*t9.*t10.*t14.*t18.*t19+l2.*t7.*t9.*t10.*t14.*t16.*t19-l2.*t5.*t14.*t18.*t19.*t20.*t21-l2.*t7.*t14.*t16.*t19.*t20.*t21)-theta_lhd.*(t44+t45+t125+t129+t176+t177+t239+t240+t241+t242+t14.*t40+t14.*t41+l1.*t3.*t19.*t20+l1.*t5.*t7.*t8.*t9-l1.*t8.*t9.*t16.*t18);
et2 = phi_lhd.*(-l1.*t9.*t14.*t19-l2.*t9.*t10.*t14.*t19+l2.*t14.*t19.*t20.*t21+l1.*t3.*t5.*t8.*t9.*t18+l1.*t3.*t7.*t8.*t9.*t16+l2.*t3.*t5.*t8.*t9.*t10.*t18+l2.*t3.*t7.*t8.*t9.*t10.*t16-l2.*t3.*t5.*t8.*t18.*t20.*t21-l2.*t3.*t7.*t8.*t16.*t20.*t21)-theta_lkd.*(t44+t45+t125+t129+t176+t177+t239+t240+t241+t242);
et3 = phi_lhd.*(t3.*t19.*t72+t3.*t19.*t73-t3.*t19.*t117-t3.*t19.*t118+l1.*t8.*t9.*t13.*t14-l1.*t2.*t3.*t5.*t7.*t9+l1.*t2.*t3.*t9.*t16.*t18+l2.*t8.*t9.*t10.*t13.*t14-l2.*t8.*t13.*t14.*t20.*t21-l2.*t2.*t3.*t5.*t7.*t9.*t10+l2.*t2.*t3.*t5.*t7.*t20.*t21+l2.*t2.*t3.*t9.*t10.*t16.*t18+l1.*t3.*t5.*t9.*t13.*t18.*t19+l1.*t3.*t7.*t9.*t13.*t16.*t19-l2.*t2.*t3.*t16.*t18.*t20.*t21)+psi_lhd.*(t48+t66+t67+t105+t106+t112+t113+t114+t131+t132+t147+t148+t149+t150+t208+t209+t216+t254+t255+t262+t264+t272+t323+t324)-theta_lkd.*(t55+t56+t138+t139+t172+t173+t190+t191+t193+t214+t257+t258+t288+t289+t292+t293+t294+t295);
et4 = -theta_lhd.*(t55+t56+t138+t139+t172+t173+t190+t191+t193+t214+t257+t258+t288+t289+t292+t293+t294+t295+t14.*t48+t14.*t112+t14.*t113+t14.*t114+l1.*t2.*t5.*t9.*t18+l1.*t2.*t7.*t9.*t16-l1.*t3.*t8.*t13.*t20+l1.*t5.*t7.*t9.*t13.*t19-l1.*t9.*t13.*t16.*t18.*t19)+thetad.*(t34-t8.*t38+t8.*t50-t8.*t70-t8.*t71+t8.*t94+t8.*t95+t8.*t119+t8.*t120-t8.*t164-t8.*t165+t8.*t220+t8.*t221+l1.*t3.*t9.*t13.*t19+l2.*t3.*t9.*t10.*t13.*t19-l2.*t3.*t13.*t19.*t20.*t21);
et5 = phid.*(t27+t32+t19.*t48+t19.*t66+t19.*t67+t19.*t105+t19.*t106+t19.*t112+t19.*t131+t19.*t132+t19.*t208+t19.*t209+t19.*t254+t19.*t255-l1.*t2.*t3.*t8.*t9+l1.*t5.*t13.*t18.*t20+l1.*t7.*t13.*t16.*t20-l2.*t2.*t3.*t8.*t9.*t10+l1.*t5.*t7.*t9.*t13.*t14+l2.*t2.*t3.*t8.*t20.*t21+l2.*t5.*t9.*t13.*t18.*t21+l2.*t5.*t10.*t13.*t18.*t20+l2.*t7.*t9.*t13.*t16.*t21+l2.*t7.*t10.*t13.*t16.*t20-l1.*t9.*t13.*t14.*t16.*t18+l2.*t5.*t7.*t9.*t10.*t13.*t14-l2.*t5.*t7.*t13.*t14.*t20.*t21-l2.*t9.*t10.*t13.*t14.*t16.*t18+l2.*t13.*t14.*t16.*t18.*t20.*t21)+psid.*(t26+t33+t48+t66+t67+t105+t106+t112+t113+t114+t131+t132+t147+t148+t149+t150+t208+t209+t216+t254+t255+t262+t264+t272+t323+t324);
et6 = -phi_lhd.*(t3.*t19.*t55+t3.*t19.*t56+t3.*t19.*t190+t3.*t19.*t191+l1.*t2.*t8.*t9.*t14+l1.*t3.*t5.*t7.*t9.*t13+l2.*t2.*t8.*t9.*t10.*t14-l1.*t3.*t9.*t13.*t16.*t18-l2.*t2.*t8.*t14.*t20.*t21+l2.*t3.*t5.*t7.*t9.*t10.*t13+l1.*t2.*t3.*t5.*t9.*t18.*t19+l1.*t2.*t3.*t7.*t9.*t16.*t19-l2.*t3.*t5.*t7.*t13.*t20.*t21-l2.*t3.*t9.*t10.*t13.*t16.*t18+l2.*t3.*t13.*t16.*t18.*t20.*t21)-psi_lhd.*(t38+t70+t71+t98+t99+t128+t164+t165+t185+t186+t196+t197+t204+t227+t228+t229+t230+t250+t252+t270+t303+t304+t318+t319);
et7 = -phid.*(t24-t35+t19.*t38+t19.*t70+t19.*t71+t19.*t128+t19.*t164+t19.*t165+t19.*t185+t19.*t186+t19.*t196+t19.*t197+t19.*t303+t19.*t304+l1.*t3.*t8.*t9.*t13+l1.*t2.*t5.*t18.*t20+l1.*t2.*t7.*t16.*t20+l1.*t2.*t5.*t7.*t9.*t14+l2.*t3.*t8.*t9.*t10.*t13+l2.*t2.*t5.*t9.*t18.*t21+l2.*t2.*t5.*t10.*t18.*t20+l2.*t2.*t7.*t9.*t16.*t21+l2.*t2.*t7.*t10.*t16.*t20-l1.*t2.*t9.*t14.*t16.*t18-l2.*t3.*t8.*t13.*t20.*t21+l2.*t2.*t5.*t7.*t9.*t10.*t14-l2.*t2.*t5.*t7.*t14.*t20.*t21-l2.*t2.*t9.*t10.*t14.*t16.*t18+l2.*t2.*t14.*t16.*t18.*t20.*t21)-psid.*(-t28+t30+t38+t70+t71+t98+t99+t128+t164+t165+t185+t186+t196+t197+t204+t227+t228+t229+t230+t250+t252+t270+t303+t304+t318+t319);
et8 = theta_lkd.*(t117+t118+t144+t145+t159+t160+t181+t200+t218+t219+t265+t266+t276+t277+t278+t279+t306+t307)+theta_lhd.*(t117+t118+t144+t145+t159+t160+t181+t200+t218+t219+t265+t266+t276+t277+t278+t279+t306+t307+t14.*t38+t14.*t98+t14.*t99+t14.*t128-l1.*t2.*t3.*t8.*t20-l1.*t5.*t9.*t13.*t18-l1.*t7.*t9.*t13.*t16+l1.*t2.*t5.*t7.*t9.*t19-l1.*t2.*t9.*t16.*t18.*t19)-thetad.*(t31+t8.*t48+t8.*t66+t8.*t67+t8.*t105+t8.*t106+t8.*t112+t8.*t131+t8.*t132+t8.*t208+t8.*t209+t8.*t254+t8.*t255+l1.*t2.*t3.*t9.*t19+l2.*t2.*t3.*t9.*t10.*t19-l2.*t2.*t3.*t19.*t20.*t21);
vel_hip_l_stance = [et1+et2,et3+et4+et5,et6+et7+et8];
if nargout > 1
    t325 = t15.*t259;
    t326 = t15.*t260;
    t327 = t15.*t261;
    et9 = -psid.*(t25+t43+t57+t91+t92+t107+t127+t179+t180+t203+t249+t268+t313)+psi_rhd.*(t43+t57+t91+t92+t107+t127+t179+t180+t203+t249+t268+t313)+thetad.*(t29+l1.*t4.*t8.*t11+l2.*t4.*t8.*t11.*t12+l1.*t6.*t7.*t19.*t22-l2.*t4.*t8.*t22.*t23+l1.*t17.*t18.*t19.*t22+l2.*t6.*t7.*t11.*t19.*t23+l2.*t6.*t7.*t12.*t19.*t22+l1.*t6.*t11.*t15.*t18.*t19-l1.*t7.*t11.*t15.*t17.*t19+l2.*t11.*t17.*t18.*t19.*t23+l2.*t12.*t17.*t18.*t19.*t22+l2.*t6.*t11.*t12.*t15.*t18.*t19-l2.*t7.*t11.*t12.*t15.*t17.*t19-l2.*t6.*t15.*t18.*t19.*t22.*t23+l2.*t7.*t15.*t17.*t19.*t22.*t23)-theta_rhd.*(t46+t47+t93+t130+t178+t199+t247+t248+t314+t315+t15.*t43+t15.*t127+l1.*t4.*t19.*t22+l1.*t6.*t7.*t8.*t11+l1.*t8.*t11.*t17.*t18);
    et10 = -phi_rhd.*(l1.*t11.*t15.*t19+l2.*t11.*t12.*t15.*t19-l2.*t15.*t19.*t22.*t23+l1.*t4.*t6.*t8.*t11.*t18-l1.*t4.*t7.*t8.*t11.*t17+l2.*t4.*t6.*t8.*t11.*t12.*t18-l2.*t4.*t7.*t8.*t11.*t12.*t17-l2.*t4.*t6.*t8.*t18.*t22.*t23+l2.*t4.*t7.*t8.*t17.*t22.*t23)-theta_rkd.*(t46+t47+t93+t130+t178+t199+t247+t248+t314+t315);
    et11 = phi_rhd.*(-t4.*t19.*t76+t4.*t19.*t77+t4.*t19.*t121-t4.*t19.*t122+l1.*t8.*t11.*t13.*t15+l1.*t2.*t4.*t6.*t7.*t11+l1.*t2.*t4.*t11.*t17.*t18+l2.*t8.*t11.*t12.*t13.*t15-l2.*t8.*t13.*t15.*t22.*t23+l2.*t2.*t4.*t6.*t7.*t11.*t12-l2.*t2.*t4.*t6.*t7.*t22.*t23+l2.*t2.*t4.*t11.*t12.*t17.*t18-l1.*t4.*t6.*t11.*t13.*t18.*t19+l1.*t4.*t7.*t11.*t13.*t17.*t19-l2.*t2.*t4.*t17.*t18.*t22.*t23)+psi_rhd.*(t37+t49+t58+t59+t68+t110+t111+t116+t141+t155+t156+t158+t163+t195+t212+t217+t256+t260+t261+t273+t290+t302+t316+t325)-theta_rkd.*(t60+t109+t133+t142+t143+t157+t174+t175+t192+t210+t211+t215+t259+t269+t300+t301+t326+t327);
    et12 = -theta_rhd.*(t60+t109+t133+t142+t143+t157+t174+t175+t192+t210+t211+t215+t259+t269+t300+t301+t326+t327+t15.*t37+t15.*t49+t15.*t116+t15.*t195+l1.*t2.*t6.*t11.*t18-l1.*t2.*t7.*t11.*t17-l1.*t4.*t8.*t13.*t22+l1.*t6.*t7.*t11.*t13.*t19+l1.*t11.*t13.*t17.*t18.*t19)-thetad.*(t34+t8.*t39+t8.*t51+t8.*t74+t8.*t75+t8.*t96+t8.*t123+t8.*t124+t8.*t169+t8.*t187+t8.*t224+t8.*t267+t8.*t305-l1.*t4.*t11.*t13.*t19-l2.*t4.*t11.*t12.*t13.*t19+l2.*t4.*t13.*t19.*t22.*t23);
    et13 = -phid.*(t27+t32+t19.*t37+t19.*t49+t19.*t58+t19.*t59+t19.*t68+t19.*t110+t19.*t111+t19.*t141+t19.*t158+t19.*t212+t19.*t256+t19.*t290+l1.*t2.*t4.*t8.*t11-l1.*t6.*t13.*t18.*t22+l1.*t7.*t13.*t17.*t22+l2.*t2.*t4.*t8.*t11.*t12+l1.*t6.*t7.*t11.*t13.*t15-l2.*t2.*t4.*t8.*t22.*t23-l2.*t6.*t11.*t13.*t18.*t23-l2.*t6.*t12.*t13.*t18.*t22+l2.*t7.*t11.*t13.*t17.*t23+l2.*t7.*t12.*t13.*t17.*t22+l1.*t11.*t13.*t15.*t17.*t18+l2.*t6.*t7.*t11.*t12.*t13.*t15-l2.*t6.*t7.*t13.*t15.*t22.*t23+l2.*t11.*t12.*t13.*t15.*t17.*t18-l2.*t13.*t15.*t17.*t18.*t22.*t23)-psid.*(t26+t33+t37+t49+t58+t59+t68+t110+t111+t116+t141+t155+t156+t158+t163+t195+t212+t217+t256+t260+t261+t273+t290+t302+t316+t325);
    et14 = phi_rhd.*(t4.*t19.*t60+t4.*t19.*t109+t4.*t19.*t133+t4.*t19.*t192-l1.*t2.*t8.*t11.*t15+l1.*t4.*t6.*t7.*t11.*t13-l2.*t2.*t8.*t11.*t12.*t15+l1.*t4.*t11.*t13.*t17.*t18+l2.*t2.*t8.*t15.*t22.*t23+l2.*t4.*t6.*t7.*t11.*t12.*t13+l1.*t2.*t4.*t6.*t11.*t18.*t19-l1.*t2.*t4.*t7.*t11.*t17.*t19-l2.*t4.*t6.*t7.*t13.*t22.*t23+l2.*t4.*t11.*t12.*t13.*t17.*t18-l2.*t4.*t13.*t17.*t18.*t22.*t23)+psi_rhd.*(t39+t51+t74+t75+t96+t100+t123+t124+t169+t187+t188+t224+t233+t234+t251+t253+t267+t281+t287+t305+t308+t309+t317+t322);
    et15 = -phid.*(-t24+t35+t19.*t39+t19.*t51+t19.*t74+t19.*t75+t19.*t96+t19.*t123+t19.*t124+t19.*t169+t19.*t187+t19.*t224+t19.*t267+t19.*t305+l1.*t4.*t8.*t11.*t13+l1.*t2.*t6.*t18.*t22-l1.*t2.*t7.*t17.*t22-l1.*t2.*t6.*t7.*t11.*t15+l2.*t4.*t8.*t11.*t12.*t13+l2.*t2.*t6.*t11.*t18.*t23+l2.*t2.*t6.*t12.*t18.*t22-l2.*t2.*t7.*t11.*t17.*t23-l2.*t2.*t7.*t12.*t17.*t22-l1.*t2.*t11.*t15.*t17.*t18-l2.*t4.*t8.*t13.*t22.*t23-l2.*t2.*t6.*t7.*t11.*t12.*t15+l2.*t2.*t6.*t7.*t15.*t22.*t23-l2.*t2.*t11.*t12.*t15.*t17.*t18+l2.*t2.*t15.*t17.*t18.*t22.*t23);
    et16 = -psid.*(t28-t30+t39+t51+t74+t75+t96+t100+t123+t124+t169+t187+t188+t224+t233+t234+t251+t253+t267+t281+t287+t305+t308+t309+t317+t322)-theta_rkd.*(t64+t65+t76+t122+t161+t170+t171+t182+t198+t222+t223+t232+t274+t282+t283+t310+t320+t321)-theta_rhd.*(t64+t65+t76+t122+t161+t170+t171+t182+t198+t222+t223+t232+t274+t282+t283+t310+t320+t321+t15.*t39+t15.*t51+t15.*t100+t15.*t188+l1.*t2.*t4.*t8.*t22+l1.*t6.*t11.*t13.*t18-l1.*t7.*t11.*t13.*t17-l1.*t2.*t6.*t7.*t11.*t19-l1.*t2.*t11.*t17.*t18.*t19)+thetad.*(t31+t8.*t37+t8.*t49+t8.*t58+t8.*t59+t8.*t68+t8.*t110+t8.*t111+t8.*t141+t8.*t158+t8.*t212+t8.*t256+t8.*t290-l1.*t2.*t4.*t11.*t19-l2.*t2.*t4.*t11.*t12.*t19+l2.*t2.*t4.*t19.*t22.*t23);
    vel_hip_r_stance = [et9+et10,et11+et12+et13,et14+et15+et16];
end
end
