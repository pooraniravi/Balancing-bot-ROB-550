%% estimate K and b
rotPerTick = 1/(20.4 * 48);
radPerTick = rotPerTick * 2*pi;
L1 = [20590.000000, -145, 0.020879
41039.000000, -287, 0.019341
61332.000000, -428, 0.021978
81666.000000, -570, 0.021319
102002.000000, -712, 0.019341
122292.000000, -854, 0.021319
142614.000000, -995, 0.023516
162933.000000, -1137, 0.023297
183293.000000, -1279, 0.019121
203612.000000, -1421, 0.022637
223961.000000, -1563, 0.023077
244252.000000, -1704, 0.020879
264653.000000, -1847, 0.020879
284999.000000, -1989, 0.023736
305293.000000, -2131, 0.023736
325613.000000, -2272, 0.021319
345932.000000, -2414, 0.018242
366281.000000, -2556, 0.021099
386572.000000, -2698, 0.021758
406892.000000, -2840, 0.020000
427239.000000, -2982, 0.021319
447532.000000, -3124, 0.024176
467852.000000, -3265, 0.021978
488172.000000, -3407, 0.017143
508574.000000, -3550, 0.019121
528892.000000, -3692, 0.021758
549211.000000, -3834, 0.020220
569534.000000, -3976, 0.019121
589853.000000, -4118, 0.021758
610174.000000, -4260, 0.023736
630492.000000, -4401, 0.022857
650812.000000, -4543, 0.018022
671132.000000, -4685, 0.020440
691451.000000, -4827, 0.021319
711772.000000, -4969, 0.020879
732092.000000, -5111, 0.019341
752600.000000, -5254, 0.021538
772894.000000, -5396, 0.023736
793224.000000, -5538, 0.022637
813525.000000, -5680, 0.019780
833851.000000, -5822, 0.019341
854172.000000, -5964, 0.021758
874491.000000, -6106, 0.020659
894812.000000, -6249, 0.019560
915130.000000, -6391, 0.020000
935452.000000, -6533, 0.022857
955772.000000, -6675, 0.023736
976092.000000, -6817, 0.020220
996480.000000, -6959, 0.019780
1016800.000000, -7101, 0.019560
1037092.000000, -7243, 0.021978
1057439.000000, -7385, 0.021319
1077760.000000, -7528, 0.020000
1098054.000000, -7670, 0.020879
1118372.000000, -7812, 0.024176
1138690.000000, -7954, 0.024396
1159011.000000, -8096, 0.018901
1179331.000000, -8238, 0.019341
1199663.000000, -8380, 0.021319
1220004.000000, -8523, 0.022857
1240293.000000, -8665, 0.020000
1260651.000000, -8807, 0.020879
1280972.000000, -8949, 0.024176
1301292.000000, -9091, 0.023956
1321642.000000, -9233, 0.022637
1341960.000000, -9375, 0.019780
1362283.000000, -9518, 0.020000
1382600.000000, -9659, 0.022198
1402932.000000, -9802, 0.022198
1423281.000000, -9944, 0.017582
1443571.000000, -10086, 0.021319
1463892.000000, -10228, 0.022637
1484213.000000, -10370, 0.023077
1504560.000000, -10513, 0.019780
1524881.000000, -10655, 0.018462
1545239.000000, -10797, 0.021099
1565533.000000, -10939, 0.022198
1585852.000000, -11081, 0.020879
1606172.000000, -11224, 0.018022
1626491.000000, -11366, 0.020879
1646813.000000, -11508, 0.023956
1667132.000000, -11650, 0.024176
1687452.000000, -11792, 0.018901
1707772.000000, -11935, 0.018022
1728092.000000, -12077, 0.021978
1748465.000000, -12220, 0.021319
1768764.000000, -12362, 0.021319
1789052.000000, -12504, 0.020000
1809372.000000, -12646, 0.022637
1829691.000000, -12788, 0.023077
1850043.000000, -12930, 0.024176
1870374.000000, -13073, 0.021319
1890692.000000, -13215, 0.018242
1911013.000000, -13357, 0.021099
1931332.000000, -13499, 0.021538
1951652.000000, -13642, 0.020659
1971972.000000, -13784, 0.017143
1992293.000000, -13927, 0.018681
2012679.000000, -14069, 0.021758];
L2 = [22525.000000, -106, 0.020000
42976.000000, -202, 0.018462
63320.000000, -298, 0.020440
83608.000000, -393, 0.018901
103928.000000, -489, 0.021758
124248.000000, -584, 0.014945
144648.000000, -680, 0.017363
164967.000000, -775, 0.016923
185287.000000, -871, 0.016484
205605.000000, -966, 0.019121
225926.000000, -1062, 0.015604
246250.000000, -1157, 0.019780
266570.000000, -1253, 0.016923
286887.000000, -1348, 0.018901
307220.000000, -1444, 0.020000
327559.000000, -1540, 0.018901
347846.000000, -1635, 0.019121
368180.000000, -1730, 0.020440
388548.000000, -1826, 0.021538
408870.000000, -1922, 0.018462
429199.000000, -2017, 0.019121
449488.000000, -2113, 0.019121
469808.000000, -2209, 0.018901
490127.000000, -2304, 0.018681
510447.000000, -2400, 0.016484
530767.000000, -2496, 0.019780
551088.000000, -2591, 0.017802
571408.000000, -2687, 0.018462
591728.000000, -2783, 0.018022
612048.000000, -2878, 0.017802
632367.000000, -2974, 0.018681
652716.000000, -3070, 0.020220
673010.000000, -3165, 0.018681
693355.000000, -3261, 0.019121
713676.000000, -3357, 0.018242
734019.000000, -3452, 0.025275
754307.000000, -3548, 0.016923
774627.000000, -3643, 0.018901
794948.000000, -3739, 0.019121
815268.000000, -3835, 0.019121
835600.000000, -3930, 0.018022
855929.000000, -4026, 0.017802
876247.000000, -4122, 0.018242
896649.000000, -4218, 0.018462
916965.000000, -4313, 0.017143
937286.000000, -4409, 0.015165
957605.000000, -4505, 0.017363
977928.000000, -4600, 0.018681
998249.000000, -4696, 0.017143
1018570.000000, -4792, 0.017363
1038924.000000, -4888, 0.018462
1059206.000000, -4983, 0.017143
1079525.000000, -5079, 0.014725
1099844.000000, -5175, 0.019560
1120166.000000, -5270, 0.017143
1140597.000000, -5366, 0.016264
1160928.000000, -5462, 0.015824
1181248.000000, -5558, 0.018462
1201567.000000, -5653, 0.020879
1221886.000000, -5749, 0.018022
1242207.000000, -5845, 0.019341
1262528.000000, -5940, 0.018901
1282847.000000, -6036, 0.018681
1303170.000000, -6131, 0.020000
1323485.000000, -6227, 0.019560
1343819.000000, -6322, 0.020440
1364119.000000, -6418, 0.020440
1384407.000000, -6513, 0.020220
1404810.000000, -6609, 0.018242
1425127.000000, -6705, 0.018681
1445449.000000, -6800, 0.017802
1465777.000000, -6896, 0.017363
1486084.000000, -6991, 0.018462
1506409.000000, -7087, 0.015604
1526767.000000, -7183, 0.018681
1547088.000000, -7278, 0.016923
1567406.000000, -7374, 0.017143
1587739.000000, -7469, 0.016923
1608039.000000, -7564, 0.018462
1628328.000000, -7660, 0.018901
1648650.000000, -7756, 0.016923
1668970.000000, -7851, 0.018901
1689290.000000, -7947, 0.019560
1709636.000000, -8042, 0.018901
1729956.000000, -8137, 0.019560
1750248.000000, -8233, 0.022857
1770569.000000, -8328, 0.016703
1790889.000000, -8424, 0.020220
1811208.000000, -8519, 0.016923
1831528.000000, -8615, 0.018022
1851847.000000, -8710, 0.018681
1872167.000000, -8806, 0.018681
1892518.000000, -8901, 0.018462
1912810.000000, -8997, 0.017802
1933127.000000, -9092, 0.018681
1953448.000000, -9188, 0.020000
1973780.000000, -9284, 0.019341
1994079.000000, -9379, 0.020659
2014395.000000, -9475, 0.018901];
R1 = [20547.000000, 129, 0.029011
40969.000000, 256, 0.024176
61289.000000, 383, 0.030110
81635.000000, 510, 0.029451
101955.000000, 637, 0.032088
122250.000000, 764, 0.033846
142567.000000, 890, 0.025055
162896.000000, 1017, 0.030110
183196.000000, 1144, 0.033407
203485.000000, 1271, 0.023077
223804.000000, 1398, 0.023736
244089.000000, 1525, 0.028132
264407.000000, 1652, 0.018242
284758.000000, 1779, 0.019121
305045.000000, 1906, 0.028132
325367.000000, 2033, 0.020440
345767.000000, 2161, 0.024615
366088.000000, 2288, 0.029670
386408.000000, 2415, 0.025934
406728.000000, 2542, 0.032088
427044.000000, 2669, 0.028791
447380.000000, 2796, 0.030330
467719.000000, 2923, 0.033407
488008.000000, 3050, 0.024396
508328.000000, 3177, 0.027033
528645.000000, 3304, 0.032747
548968.000000, 3431, 0.022198
569290.000000, 3558, 0.025714
589636.000000, 3685, 0.029231
609955.000000, 3812, 0.018022
630247.000000, 3939, 0.017802
650569.000000, 4066, 0.028132
670888.000000, 4193, 0.021319
691209.000000, 4320, 0.024396
711528.000000, 4447, 0.030769
731848.000000, 4575, 0.026154
752167.000000, 4702, 0.032747
772487.000000, 4829, 0.028352
792807.000000, 4957, 0.030549
813129.000000, 5084, 0.033187
833477.000000, 5211, 0.024835
853770.000000, 5338, 0.027253
874090.000000, 5466, 0.032088
894437.000000, 5593, 0.024615
914740.000000, 5720, 0.025714
935028.000000, 5847, 0.032527
955348.000000, 5974, 0.021538
975680.000000, 6101, 0.023297
995968.000000, 6229, 0.028352
1016288.000000, 6356, 0.016923
1036607.000000, 6483, 0.019780
1056925.000000, 6610, 0.027912
1077209.000000, 6737, 0.020659
1097570.000000, 6865, 0.021978
1117888.000000, 6992, 0.029670
1138209.000000, 7120, 0.024176
1158530.000000, 7247, 0.028791
1178849.000000, 7374, 0.029670
1199170.000000, 7501, 0.029451
1219488.000000, 7629, 0.035385
1239808.000000, 7756, 0.027473
1260128.000000, 7884, 0.029890
1280448.000000, 8011, 0.032308
1300768.000000, 8139, 0.024615
1321088.000000, 8266, 0.028352
1341408.000000, 8394, 0.032308
1361796.000000, 8521, 0.024176
1382098.000000, 8649, 0.026154
1402387.000000, 8776, 0.032747
1422707.000000, 8903, 0.023077
1443029.000000, 9031, 0.025495
1463358.000000, 9158, 0.029890
1483650.000000, 9286, 0.020220
1503965.000000, 9413, 0.020000
1524318.000000, 9541, 0.027692
1544649.000000, 9668, 0.017363
1564982.000000, 9795, 0.018901
1585319.000000, 9923, 0.026813
1605610.000000, 10050, 0.018681
1625940.000000, 10178, 0.018901
1646270.000000, 10306, 0.028352
1666589.000000, 10433, 0.022198
1686909.000000, 10560, 0.024835
1707228.000000, 10688, 0.030110
1727545.000000, 10815, 0.025714
1747869.000000, 10943, 0.028132
1768198.000000, 11070, 0.028791
1788485.000000, 11197, 0.029231
1808806.000000, 11325, 0.034945
1829126.000000, 11452, 0.026813
1849557.000000, 11580, 0.029670
1869846.000000, 11708, 0.033407
1890168.000000, 11835, 0.025495
1910516.000000, 11963, 0.027912
1930805.000000, 12090, 0.032088
1951128.000000, 12217, 0.024396
1971449.000000, 12345, 0.029011
1991769.000000, 12473, 0.031868
2012087.000000, 12600, 0.023077];
R2 = [20589.000000, 70, 0.019341
41020.000000, 139, 0.019560
61315.000000, 208, 0.018901
81633.000000, 276, 0.018242
101953.000000, 345, 0.018462
122273.000000, 414, 0.019780
142592.000000, 483, 0.019560
162913.000000, 552, 0.018901
183232.000000, 621, 0.019560
203552.000000, 690, 0.018901
223875.000000, 758, 0.019341
244226.000000, 827, 0.018681
264581.000000, 896, 0.019121
284940.000000, 965, 0.019121
305260.000000, 1034, 0.020000
325554.000000, 1103, 0.018681
345901.000000, 1172, 0.019341
366194.000000, 1240, 0.018901
386513.000000, 1309, 0.019341
406834.000000, 1378, 0.019560
427151.000000, 1447, 0.019780
447473.000000, 1516, 0.019780
467793.000000, 1585, 0.019560
488180.000000, 1654, 0.019341
508483.000000, 1723, 0.018901
528773.000000, 1792, 0.019341
549093.000000, 1860, 0.019780
569415.000000, 1929, 0.018901
589733.000000, 1998, 0.019780
610052.000000, 2067, 0.019780
630383.000000, 2136, 0.018242
650673.000000, 2205, 0.019341
670993.000000, 2274, 0.020220
691343.000000, 2343, 0.020440
711633.000000, 2412, 0.019121
731954.000000, 2480, 0.019341
752275.000000, 2549, 0.018901
772595.000000, 2618, 0.019341
792925.000000, 2687, 0.019341
813226.000000, 2756, 0.019560
833553.000000, 2825, 0.018901
853872.000000, 2895, 0.018242
874193.000000, 2964, 0.019780
894512.000000, 3033, 0.018901
914833.000000, 3102, 0.019341
935153.000000, 3171, 0.018901
955474.000000, 3240, 0.018901
975863.000000, 3309, 0.018901
996155.000000, 3378, 0.019121
1016474.000000, 3448, 0.019121
1036794.000000, 3517, 0.020000
1057114.000000, 3586, 0.018681
1077433.000000, 3655, 0.018901
1097753.000000, 3724, 0.019341
1118072.000000, 3794, 0.019121
1138394.000000, 3863, 0.018901
1158714.000000, 3932, 0.020440
1179033.000000, 4001, 0.018901
1199352.000000, 4070, 0.018901
1219672.000000, 4139, 0.018901
1240020.000000, 4209, 0.018681
1260312.000000, 4278, 0.018462
1280633.000000, 4347, 0.018681
1300955.000000, 4416, 0.019341
1321284.000000, 4485, 0.019121
1341585.000000, 4555, 0.018681
1362033.000000, 4624, 0.019341
1382352.000000, 4694, 0.019560
1402673.000000, 4763, 0.019121
1422990.000000, 4832, 0.018901
1443313.000000, 4901, 0.018901
1463632.000000, 4971, 0.019341
1483980.000000, 5040, 0.019121
1504275.000000, 5109, 0.019121
1524593.000000, 5179, 0.018901
1544912.000000, 5248, 0.019121
1565233.000000, 5317, 0.019121
1585554.000000, 5387, 0.018462
1605874.000000, 5456, 0.018681
1626193.000000, 5525, 0.019341
1646514.000000, 5595, 0.019780
1666834.000000, 5664, 0.019341
1687152.000000, 5734, 0.019341
1707473.000000, 5803, 0.019341
1727862.000000, 5873, 0.019341
1748181.000000, 5942, 0.019341
1768473.000000, 6011, 0.019341
1788792.000000, 6081, 0.018242
1809112.000000, 6150, 0.019341
1829433.000000, 6219, 0.019560
1849784.000000, 6289, 0.018462
1870075.000000, 6358, 0.019121
1890393.000000, 6428, 0.019560
1910745.000000, 6497, 0.019780
1931043.000000, 6566, 0.019341
1951343.000000, 6636, 0.019121
1971633.000000, 6705, 0.019780
1991955.000000, 6775, 0.018901
2012300.000000, 6844, 0.019780];

VL = [ones(size(L1, 1), 1)*11.75; ones(size(L2, 1), 1)*7.90];
VR = [ones(size(R1, 1), 1)*11.73; ones(size(R2, 1), 1)*6.25];

RL = 4.8;
RR = 6.2;
% V - RI = WK
YL = VL - [L1(:,3); L2(:,3)].*RL;
WL = [L1(:,2)./(L1(:,1)/1e6); L2(:,2)./(L2(:,1)/1e6)] .* -radPerTick;
KL = WL \ YL;
% estimate variance
YhatL = WL .* KL;
EL = YL - YhatL;
% standard error of regression
sL = sqrt(EL'*EL/(size(EL,1)-1));

YR = VR - [R1(:,3); R2(:,3)].*RR;
WR = [R1(:,2)./(R1(:,1)/1e6); R2(:,2)./(R2(:,1)/1e6)] .* radPerTick;
KR = WR \ YR;
YhatR = WR .* KR;
ER = YR - YhatR;
sR = sqrt(ER'*ER/(size(ER,1)-1));

fprintf('Left K: %f +/- %f\n', KL, sL);
fprintf('Right K: %f +/- %f\n', KR, sR);

%% steady state noload
LNL = WL(1:size(L1,1));
LNR = WR(1:size(R1,1));
wNLL = mean(LNL);
wNLR = mean(LNR);
fprintf('Left w_NL: %f +/- %f\n', wNLL, std(LNL));
fprintf('Right w_NL: %f +/- %f\n', wNLR, std(LNR));

%% friction coefficient
bL = (KL*VL(1)/wNLL - KL^2)/RL;
bLmin = ((KL+sL)*VL(1)/wNLL - (KL+sL)^2)/RL;
bLmax = ((KL-sL)*VL(1)/wNLL - (KL-sL)^2)/RL;

bR = (KR*VR(1)/wNLR - KR^2)/RR;
bRmin = ((KR+sR)*VR(1)/wNLR - (KR+sR)^2)/RR;
bRmax = ((KR-sR)*VR(1)/wNLR - (KR-sR)^2)/RR;
fprintf('Left b: %f +/- %f\n', bL, max(bLmax-bL, bL-bLmin));
fprintf('Right b: %f +/- %f\n', bR, max(bRmax-bR, bR-bRmin));

%% estimate J
L3=[5.000000, -2, 0.014505
20668.000000, -144, 0.012967
41123.000000, -279, 0.011868
61451.000000, -407, 0.008571
81769.000000, -529, 0.008791
102119.000000, -646, 0.007473
122423.000000, -758, 0.007253
142749.000000, -866, 0.006154
163081.000000, -969, 0.005055
183409.000000, -1068, 0.005275
203729.000000, -1163, 0.002637
224050.000000, -1254, 0.005495
244410.000000, -1340, 0.003077
264757.000000, -1424, 0.005275
285051.000000, -1503, 0.003077
305368.000000, -1579, 0.005055
325688.000000, -1652, 0.003297
346008.000000, -1721, 0.004396
366328.000000, -1787, 0.003297
386659.000000, -1850, 0.004396
406948.000000, -1910, 0.003077
427297.000000, -1967, 0.002857
447599.000000, -2021, 0.003297
467890.000000, -2073, 0.003516
488209.000000, -2121, 0.005055
508571.000000, -2167, 0.003956
528891.000000, -2210, 0.005275
549211.000000, -2251, 0.005275
569845.000000, -2290, 0.003077
590182.000000, -2326, 0.003297
610509.000000, -2360, 0.003077
630828.000000, -2392, 0.005055
651148.000000, -2421, 0.003297
671469.000000, -2448, 0.003077
691788.000000, -2473, 0.005055
712109.000000, -2497, 0.003956
732409.000000, -2518, 0.004396
752761.000000, -2537, 0.005495
773077.000000, -2555, 0.003956
793370.000000, -2571, 0.003297
813690.000000, -2585, 0.004835
834011.000000, -2598, 0.005055
854329.000000, -2609, 0.003297
874649.000000, -2618, 0.005275
894969.000000, -2626, 0.004396
915289.000000, -2633, 0.003297
935636.000000, -2637, 0.003297
955929.000000, -2641, 0.005055];

R3=[4.000000, 3, 0.044615
20565.000000, 130, 0.004835
40981.000000, 253, 0.005055
61307.000000, 371, 0.004176
81625.000000, 486, 0.004615
101947.000000, 597, 0.004835
122267.000000, 705, 0.004396
142586.000000, 810, 0.004396
162907.000000, 911, 0.004615
183226.000000, 1009, 0.004615
203546.000000, 1103, 0.005055
223921.000000, 1195, 0.004615
244289.000000, 1284, 0.004615
264660.000000, 1369, 0.004835
284987.000000, 1451, 0.004396
305308.000000, 1529, 0.004396
325627.000000, 1605, 0.004396
345945.000000, 1678, 0.004396
366268.000000, 1748, 0.004835
386587.000000, 1816, 0.004396
406906.000000, 1881, 0.004835
427227.000000, 1943, 0.003956
447543.000000, 2002, 0.003956
467828.000000, 2059, 0.005055
488188.000000, 2114, 0.005055
508508.000000, 2165, 0.004396
528829.000000, 2215, 0.005055
549147.000000, 2262, 0.004176
569466.000000, 2306, 0.004835
589789.000000, 2349, 0.003956
610139.000000, 2389, 0.004176
630468.000000, 2426, 0.004835
650786.000000, 2462, 0.004176
671107.000000, 2495, 0.004176
691427.000000, 2527, 0.003956
711746.000000, 2556, 0.004396
732069.000000, 2583, 0.004176
752388.000000, 2608, 0.004835
772706.000000, 2632, 0.004615
793027.000000, 2653, 0.004835
813378.000000, 2673, 0.004176
833669.000000, 2691, 0.004396
854014.000000, 2708, 0.004176
874307.000000, 2722, 0.005055
894626.000000, 2735, 0.004835
914975.000000, 2747, 0.004835
935268.000000, 2757, 0.005055
955588.000000, 2765, 0.004835
975935.000000, 2772, 0.004615
996228.000000, 2777, 0.004835
1016549.000000, 2781, 0.004396
1036894.000000, 2783, 0.004615
1057198.000000, 2786, 0.004835];

%caluclate the instant w and a 
 WLC=ones(size(L3,1),1);
 ALC=ones(size(L3,1),1);
 WRC=ones(size(R3,1),1);
 ARC=ones(size(R3,1),1);
% WLC(1)=L3(1,2)./(L3(1,1)/1e6).* -radPerTick;
% ALC(1)=WLC(1)./(L3(1,1)/1e6).* -radPerTick;
% WRC(1)=R3(1,2)./(R3(1,1)/1e6).* radPerTick;
% ARC(1)=WRC(1)./(R3(1,1)/1e6).* radPerTick;
 for i=1:size(L3,1)-1
 WLC(i) = (L3(i+1,2)-L3(i,2))/((L3(i+1,1)-L3(i,1))/1e6) * -radPerTick;
 ALC(i) = -(WLC(i+1)-WLC(i))/((L3(i+1,1)-L3(i,1))/1e6);
 end
 for i=1:size(R3,1)-1
 WRC(i) = (R3(i+1,2)-R3(i,2))./((R3(i+1,1)-R3(i,1))/1e6) .* radPerTick;
 ARC(i) = -(WRC(i+1)-WRC(i))/((R3(i+1,1)-R3(i,1))/1e6);
 end

%calculate the average w and a
%WLC = L3(:,2)./(L3(:,1)/1e6).* -radPerTick;
%WRC = R3(:,2)./(R3(:,1)/1e6).* radPerTick;
%ALC = L3(:,2)./(L3(:,1)/1e6)./(L3(:,1)/1e6).* -radPerTick;
%ARC = R3(:,2)./(R3(:,1)/1e6)./(R3(:,1)/1e6).* radPerTick;

IL=bL*WLC./ALC;
IR=bR*WRC./ARC;
fprintf('Left Inertia: %f +/- %f\n', mean(IL), std(IL));
fprintf('Right Inertia: %f +/- %f\n', mean(IR), std(IR));