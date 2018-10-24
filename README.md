```
/*******************************************************************************
*                           Balancebot Template Code
*                           pgaskell@umich.edu
*       
*******************************************************************************/

bin/			             : Binaries folder
balanceebot/balanceebot.c/.h : Main setup and threads
test_motors/test_motors.c/.h : Program to test motor implementation
common/mb_controller.c/.h    : Contoller for manual and autonomous nav
common/mb_defs.h             : Define hardware config
common/mb_motors.c/.h        : Motor functions to be used by balancebot
common/mb_odometry.c/.h	     : Odometry functions
```

## Sections to split up
- balance controller (section 2) - Phoneix
- odometry and motion control (section 3) - Johnson (will provide limited help for other sections)
- path planning and following (section 4) -

## How to automatically sync code from computer to Beaglebone:
1. (on computer) ssh-keygen to default `id_rsa` and `id_rsa.pub`
2. (on bbg) copy content of computer's `id_rsa.pub` to bbg's `~/.ssh/authorized_keys` (create file/directory if necessary)
3. (on computer) `python -m --user pip install watchdog`
4. download script to computer: https://gist.github.com/LemonPi/49e749489a46186665ea4a5e5e0819c1
5. run script passing in path to directory to sync `python watch_sync.py ubalance/`

## Determining motor parameters
- in steady state V = Ri + Kw
    - start measuring after spinning for 2s to ensure steady state
    - measure V across motors
    - measure i using current sense
    - measure w using encoder
    - get multiple data points and estimate R and K using linear regression
        - V = [i w][R K]'
        - [V; ...; V_N] = [i_1 w_1; ...; i_N w_N][R K]'
            - V = Ix
        - [R K]' = pseudoInv(I)V
- steady state no load speed ω_NL = KV/(K^2 + Rb)
    - estimate b = (KV/(ω_NL) - K^2) / R
    - ignore c for modelling
- determine stall torque by holding onto wheel to stall it
    - T_s = KV/R
    - measure voltage across motors
- determine inertia J by accelerating wheel with constant current
    - Ki = Jw_dot + bw
    - measure w_dot by tracking w over time
### No load speed
left motor
```asm
30.000000, -1, 0.105495
20606.000000, -143, 0.105275
41071.000000, -285, 0.101538
61360.000000, -425, 0.103077
81689.000000, -566, 0.103736
102010.000000, -707, 0.105934
122301.000000, -847, 0.104176
142621.000000, -988, 0.101978
162951.000000, -1129, 0.104835
183239.000000, -1270, 0.103956
203575.000000, -1411, 0.103516
223911.000000, -1551, 0.102637
244199.000000, -1692, 0.103297
264521.000000, -1833, 0.105934
284874.000000, -1974, 0.105714
305200.000000, -2115, 0.103516
325521.000000, -2256, 0.103077
345841.000000, -2397, 0.103516
366159.000000, -2537, 0.104835
386479.000000, -2678, 0.104396
406798.000000, -2819, 0.102637
427116.000000, -2959, 0.103297
447399.000000, -3100, 0.103516
467719.000000, -3241, 0.106593
488038.000000, -3382, 0.104615
508358.000000, -3522, 0.101978
528806.000000, -3664, 0.104176
549101.000000, -3805, 0.103736
569420.000000, -3946, 0.104176
589741.000000, -4086, 0.101538
610061.000000, -4227, 0.104176
630380.000000, -4368, 0.105275
650696.000000, -4509, 0.104615
670990.000000, -4650, 0.103516
691279.000000, -4791, 0.103297
711598.000000, -4931, 0.103297
731920.000000, -5072, 0.104396
752239.000000, -5213, 0.103736
772558.000000, -5354, 0.102418
792881.000000, -5495, 0.102637
813199.000000, -5636, 0.103736
833518.000000, -5777, 0.105714
860630.000000, -5973, 0.103736
882091.000000, -6114, 0.104176
902568.000000, -6256, 0.105495
922899.000000, -6397, 0.102418
943219.000000, -6538, 0.102198
963539.000000, -6678, 0.104396
983859.000000, -6819, 0.106374
1004179.000000, -6960, 0.103956
1024499.000000, -7101, 0.103077
1045047.000000, -7244, 0.103297
1065340.000000, -7385, 0.104396
1085657.000000, -7526, 0.104396
1105987.000000, -7666, 0.102637
1126276.000000, -7807, 0.103956
1146596.000000, -7949, 0.102198
1166916.000000, -8090, 0.104615
1187196.000000, -8231, 0.103516
1207532.000000, -8372, 0.102857
1227867.000000, -8513, 0.103736
1248158.000000, -8654, 0.104176
1268510.000000, -8795, 0.105934
1289039.000000, -8938, 0.103736
1309359.000000, -9079, 0.103736
1329711.000000, -9221, 0.103736
1350028.000000, -9362, 0.106593
1370359.000000, -9503, 0.104835
1390681.000000, -9645, 0.101319
1410999.000000, -9786, 0.102857
1431319.000000, -9927, 0.103956
1451638.000000, -10068, 0.106154
1471987.000000, -10210, 0.102857
1492319.000000, -10351, 0.102198
1512639.000000, -10493, 0.103297
1533069.000000, -10635, 0.105055
1553386.000000, -10776, 0.103736
1573679.000000, -10917, 0.102418
1594000.000000, -11059, 0.102198
1614318.000000, -11200, 0.103297
1634639.000000, -11341, 0.105714
1654959.000000, -11482, 0.101758
1675279.000000, -11624, 0.103956
1695599.000000, -11765, 0.103956
1715919.000000, -11906, 0.104835
1736239.000000, -12047, 0.103956
1756559.000000, -12188, 0.102198
1776879.000000, -12330, 0.103956
1797230.000000, -12471, 0.103736
1817520.000000, -12612, 0.104835
1837866.000000, -12754, 0.103736
1858190.000000, -12895, 0.102418
1878508.000000, -13036, 0.104176
1898800.000000, -13178, 0.103956
1919120.000000, -13319, 0.103736
1939440.000000, -13460, 0.103516
1959759.000000, -13602, 0.103077
1980079.000000, -13743, 0.104176
2000409.000000, -13884, 0.104396
```
11.69V

```asm
6.000000, -1, 0.104396
20564.000000, -116, 0.103956
40983.000000, -231, 0.102857
61319.000000, -345, 0.104615
81610.000000, -458, 0.105055
101933.000000, -573, 0.104835
122250.000000, -686, 0.104615
142569.000000, -800, 0.103516
162891.000000, -914, 0.104176
183224.000000, -1028, 0.105714
203553.000000, -1142, 0.107473
223882.000000, -1256, 0.105714
244171.000000, -1370, 0.103077
264491.000000, -1484, 0.104396
284864.000000, -1598, 0.106593
305308.000000, -1713, 0.104396
325672.000000, -1827, 0.102198
345987.000000, -1941, 0.102857
366320.000000, -2055, 0.106593
386610.000000, -2169, 0.106593
406933.000000, -2283, 0.104176
427278.000000, -2397, 0.103077
447600.000000, -2511, 0.102857
467888.000000, -2624, 0.106374
488239.000000, -2738, 0.106593
508586.000000, -2853, 0.103077
528912.000000, -2967, 0.104396
549271.000000, -3081, 0.105055
569591.000000, -3195, 0.107033
589911.000000, -3309, 0.104615
610231.000000, -3423, 0.102418
630551.000000, -3537, 0.104176
650871.000000, -3650, 0.106374
671192.000000, -3765, 0.105714
691511.000000, -3878, 0.103077
711833.000000, -3993, 0.101538
732150.000000, -4107, 0.103516
752470.000000, -4221, 0.105714
772790.000000, -4335, 0.104176
793109.000000, -4449, 0.101758
813430.000000, -4563, 0.103736
833753.000000, -4677, 0.105934
854111.000000, -4791, 0.107033
874431.000000, -4905, 0.104396
894751.000000, -5019, 0.102418
915071.000000, -5133, 0.104615
935392.000000, -5247, 0.105275
955712.000000, -5361, 0.104615
976032.000000, -5475, 0.104615
996353.000000, -5590, 0.099341
1016682.000000, -5704, 0.104396
1036972.000000, -5818, 0.107912
1057291.000000, -5932, 0.105275
1077608.000000, -6046, 0.102857
1097930.000000, -6160, 0.103516
1118262.000000, -6274, 0.103956
1138562.000000, -6388, 0.105275
1158850.000000, -6502, 0.104835
1179183.000000, -6616, 0.103077
1199480.000000, -6730, 0.104176
1219771.000000, -6844, 0.107912
1240092.000000, -6958, 0.103297
1260412.000000, -7072, 0.103077
1280733.000000, -7186, 0.103956
1301052.000000, -7300, 0.104835
1321369.000000, -7414, 0.105934
1341688.000000, -7528, 0.102637
1362008.000000, -7642, 0.103516
1382329.000000, -7756, 0.104615
1402647.000000, -7870, 0.105934
1422927.000000, -7984, 0.105934
1443211.000000, -8098, 0.102418
1463532.000000, -8212, 0.103516
1483848.000000, -8326, 0.106374
1504184.000000, -8440, 0.106593
1524481.000000, -8554, 0.103297
1544770.000000, -8668, 0.103077
1565118.000000, -8782, 0.103956
1585409.000000, -8896, 0.105275
1605731.000000, -9010, 0.104176
1626051.000000, -9124, 0.103516
1646370.000000, -9238, 0.103077
1667164.000000, -9354, 0.104176
1687490.000000, -9469, 0.105934
1707811.000000, -9582, 0.103516
1728130.000000, -9696, 0.103736
1748450.000000, -9810, 0.103956
1768770.000000, -9924, 0.106813
1789088.000000, -10038, 0.103736
1809412.000000, -10152, 0.102857
1829769.000000, -10266, 0.105055
1850093.000000, -10380, 0.104835
1870410.000000, -10493, 0.104396
1890730.000000, -10607, 0.103297
1911080.000000, -10721, 0.103956
1931373.000000, -10835, 0.104396
1951692.000000, -10949, 0.105495
1972012.000000, -11063, 0.105055
1992332.000000, -11177, 0.102857
2012650.000000, -11291, 0.102857
```
9.17V

```asm
5.000000, -1, 0.099780
20658.000000, -72, 0.105495
41044.000000, -143, 0.100000
61372.000000, -214, 0.104615
81690.000000, -285, 0.106593
102011.000000, -355, 0.103077
122331.000000, -426, 0.104176
142650.000000, -497, 0.105495
162978.000000, -567, 0.106593
183281.000000, -638, 0.111209
203570.000000, -708, 0.102198
223891.000000, -779, 0.104176
244210.000000, -850, 0.105275
264598.000000, -920, 0.108132
284891.000000, -991, 0.104835
305241.000000, -1062, 0.103956
325531.000000, -1132, 0.104176
345853.000000, -1203, 0.105495
366171.000000, -1273, 0.105495
386492.000000, -1344, 0.100659
406810.000000, -1415, 0.102857
427159.000000, -1485, 0.106374
447450.000000, -1556, 0.105934
467770.000000, -1627, 0.104396
488090.000000, -1697, 0.102418
508438.000000, -1768, 0.105495
528733.000000, -1839, 0.105275
549051.000000, -1910, 0.105275
569371.000000, -1980, 0.103516
589689.000000, -2051, 0.103736
610010.000000, -2121, 0.105714
630331.000000, -2192, 0.107033
650678.000000, -2263, 0.103956
670970.000000, -2334, 0.102198
691291.000000, -2404, 0.104835
711611.000000, -2475, 0.105055
731932.000000, -2546, 0.107253
752250.000000, -2617, 0.103077
772613.000000, -2688, 0.104176
792933.000000, -2758, 0.104396
813252.000000, -2829, 0.106813
833573.000000, -2900, 0.105275
853891.000000, -2970, 0.104176
874210.000000, -3041, 0.103956
894532.000000, -3112, 0.102637
914851.000000, -3183, 0.107912
935171.000000, -3253, 0.103297
955490.000000, -3324, 0.103516
975838.000000, -3395, 0.107033
996130.000000, -3465, 0.108571
1016450.000000, -3536, 0.107253
1036773.000000, -3607, 0.102857
1057093.000000, -3678, 0.103077
1077442.000000, -3749, 0.106593
1097733.000000, -3819, 0.108132
1118053.000000, -3890, 0.103736
1138371.000000, -3961, 0.103736
1158691.000000, -4032, 0.110769
1179011.000000, -4102, 0.105714
1199331.000000, -4173, 0.105714
1219651.000000, -4244, 0.102637
1239970.000000, -4314, 0.103956
1260319.000000, -4385, 0.105055
1280612.000000, -4456, 0.106593
1300931.000000, -4526, 0.104396
1321252.000000, -4597, 0.102637
1341570.000000, -4668, 0.105055
1361890.000000, -4739, 0.106374
1382213.000000, -4809, 0.106813
1402541.000000, -4880, 0.102857
1422830.000000, -4951, 0.104615
1443150.000000, -5021, 0.104835
1463471.000000, -5092, 0.106154
1483792.000000, -5163, 0.104176
1504111.000000, -5234, 0.102198
1524412.000000, -5304, 0.104176
1544743.000000, -5375, 0.106154
1565041.000000, -5446, 0.106154
1585331.000000, -5517, 0.109670
1605683.000000, -5587, 0.100879
1626013.000000, -5658, 0.105934
1646359.000000, -5729, 0.107692
1666652.000000, -5800, 0.104176
1687002.000000, -5870, 0.102418
1707291.000000, -5941, 0.106813
1727640.000000, -6012, 0.106813
1747930.000000, -6083, 0.107692
1768251.000000, -6154, 0.103297
1788612.000000, -6224, 0.103516
1808942.000000, -6295, 0.103956
1829242.000000, -6366, 0.105934
1849558.000000, -6437, 0.105934
1869878.000000, -6507, 0.103736
1890171.000000, -6578, 0.106813
1910492.000000, -6649, 0.105714
1930811.000000, -6720, 0.105055
1951130.000000, -6790, 0.103077
1971451.000000, -6861, 0.105495
1991773.000000, -6932, 0.103736
2012092.000000, -7003, 0.105495
```
5.94V

right motor
```asm
5.000000, 0, 0.097802
20552.000000, 128, 0.098901
40970.000000, 254, 0.096703
61291.000000, 380, 0.095824
81611.000000, 506, 0.100000
101932.000000, 632, 0.098462
122252.000000, 758, 0.103956
142569.000000, 884, 0.096923
162890.000000, 1010, 0.099780
183212.000000, 1136, 0.100220
203533.000000, 1262, 0.095604
223852.000000, 1388, 0.096264
244171.000000, 1514, 0.100220
264492.000000, 1640, 0.097143
284810.000000, 1766, 0.101538
305159.000000, 1892, 0.096923
325451.000000, 2018, 0.100879
345771.000000, 2144, 0.100440
366090.000000, 2270, 0.094505
386411.000000, 2395, 0.095824
406730.000000, 2522, 0.100220
427050.000000, 2648, 0.100220
447409.000000, 2774, 0.101538
467769.000000, 2900, 0.097363
488092.000000, 3026, 0.098901
508438.000000, 3153, 0.100659
528763.000000, 3279, 0.095604
549052.000000, 3405, 0.094945
569371.000000, 3531, 0.098022
589719.000000, 3657, 0.098681
610049.000000, 3783, 0.100220
630605.000000, 3911, 0.098022
650931.000000, 4038, 0.101319
671253.000000, 4164, 0.100659
691572.000000, 4290, 0.098462
711892.000000, 4416, 0.096264
732210.000000, 4542, 0.098242
752532.000000, 4668, 0.094505
772848.000000, 4794, 0.099341
793171.000000, 4920, 0.100440
813491.000000, 5046, 0.101099
833810.000000, 5172, 0.100659
854141.000000, 5297, 0.097363
874440.000000, 5423, 0.097802
894731.000000, 5549, 0.100000
915051.000000, 5676, 0.096484
935408.000000, 5802, 0.098242
955762.000000, 5928, 0.098901
976063.000000, 6054, 0.102198
996363.000000, 6179, 0.100220
1016693.000000, 6305, 0.097582
1037013.000000, 6431, 0.096264
1057329.000000, 6557, 0.099560
1077661.000000, 6683, 0.095604
1097999.000000, 6809, 0.097363
1118289.000000, 6935, 0.098462
1138607.000000, 7061, 0.100659
1158929.000000, 7187, 0.100659
1179251.000000, 7313, 0.097363
1199571.000000, 7439, 0.094725
1219920.000000, 7565, 0.099560
1240209.000000, 7690, 0.096264
1260530.000000, 7816, 0.099780
1280851.000000, 7942, 0.099341
1301198.000000, 8068, 0.097802
1321489.000000, 8194, 0.101099
1341811.000000, 8319, 0.097143
1362130.000000, 8445, 0.096703
1382450.000000, 8571, 0.096923
1402771.000000, 8697, 0.098462
1423121.000000, 8822, 0.103736
1443441.000000, 8948, 0.099121
1463732.000000, 9074, 0.098681
1484051.000000, 9200, 0.099341
1504373.000000, 9325, 0.095604
1524692.000000, 9451, 0.095824
1545012.000000, 9577, 0.100659
1565331.000000, 9702, 0.099560
1585651.000000, 9828, 0.100220
1605972.000000, 9954, 0.098022
1626291.000000, 10079, 0.096703
1646639.000000, 10205, 0.099341
1666930.000000, 10331, 0.093846
1687252.000000, 10457, 0.098681
1707570.000000, 10583, 0.100220
1727890.000000, 10708, 0.099121
1748242.000000, 10834, 0.101099
1768530.000000, 10960, 0.095165
1788852.000000, 11085, 0.095604
1809181.000000, 11211, 0.099341
1829471.000000, 11337, 0.098242
1849791.000000, 11462, 0.101538
1870111.000000, 11588, 0.096923
1890431.000000, 11714, 0.099560
1910751.000000, 11840, 0.101319
1931107.000000, 11966, 0.094945
1951402.000000, 12091, 0.096923
1971718.000000, 12217, 0.100220
1992011.000000, 12343, 0.100220
2012492.000000, 12470, 0.102198
```
11.70V

```asm
6.000000, 1, 0.097802
20621.000000, 106, 0.095604
40948.000000, 210, 0.100440
61269.000000, 314, 0.095604
81585.000000, 418, 0.098462
101920.000000, 522, 0.092088
122210.000000, 626, 0.100659
142539.000000, 730, 0.096703
162827.000000, 834, 0.098681
183147.000000, 938, 0.097363
203478.000000, 1043, 0.099780
223767.000000, 1147, 0.097363
244090.000000, 1251, 0.097143
264410.000000, 1355, 0.094505
284728.000000, 1459, 0.096264
305076.000000, 1563, 0.095824
325367.000000, 1667, 0.094066
345686.000000, 1772, 0.096484
366038.000000, 1876, 0.099341
386358.000000, 1980, 0.096703
406650.000000, 2084, 0.097582
426996.000000, 2188, 0.097363
447288.000000, 2292, 0.095385
467608.000000, 2397, 0.090549
487928.000000, 2501, 0.094725
508248.000000, 2605, 0.095604
528567.000000, 2709, 0.097582
548889.000000, 2813, 0.095165
569207.000000, 2917, 0.097363
589529.000000, 3021, 0.097802
609845.000000, 3125, 0.102198
630129.000000, 3230, 0.093626
650490.000000, 3334, 0.098022
670808.000000, 3438, 0.096044
691156.000000, 3542, 0.097582
711450.000000, 3646, 0.098901
731778.000000, 3750, 0.102198
752068.000000, 3854, 0.094945
772388.000000, 3959, 0.090549
792724.000000, 4063, 0.090989
813047.000000, 4167, 0.100220
833367.000000, 4271, 0.094945
853688.000000, 4375, 0.097143
874020.000000, 4480, 0.098022
894330.000000, 4584, 0.096484
914649.000000, 4688, 0.095385
934968.000000, 4792, 0.095165
955287.000000, 4896, 0.100659
975609.000000, 5000, 0.095604
995927.000000, 5105, 0.097143
1016276.000000, 5209, 0.096484
1036596.000000, 5313, 0.093187
1056929.000000, 5418, 0.102637
1077248.000000, 5522, 0.098681
1097568.000000, 5626, 0.098462
1117890.000000, 5730, 0.100000
1138209.000000, 5834, 0.099121
1158538.000000, 5939, 0.097363
1178838.000000, 6043, 0.097143
1199130.000000, 6147, 0.096923
1219450.000000, 6251, 0.093187
1239765.000000, 6356, 0.098901
1260085.000000, 6460, 0.098901
1280416.000000, 6564, 0.099780
1300715.000000, 6668, 0.094505
1321005.000000, 6773, 0.099780
1341324.000000, 6877, 0.098242
1361619.000000, 6981, 0.097802
1381956.000000, 7085, 0.096703
1402479.000000, 7190, 0.101758
1422767.000000, 7295, 0.097143
1443088.000000, 7399, 0.097582
1463407.000000, 7503, 0.095385
1483727.000000, 7607, 0.096264
1504045.000000, 7712, 0.095165
1524367.000000, 7816, 0.092747
1544687.000000, 7920, 0.099341
1565007.000000, 8025, 0.097802
1585328.000000, 8129, 0.096264
1605645.000000, 8233, 0.097143
1625968.000000, 8337, 0.097363
1646329.000000, 8442, 0.103077
1666649.000000, 8546, 0.095824
1686970.000000, 8650, 0.101758
1707290.000000, 8755, 0.100000
1727635.000000, 8859, 0.096484
1747927.000000, 8963, 0.096044
1768248.000000, 9068, 0.096264
1788568.000000, 9172, 0.092967
1808889.000000, 9276, 0.098462
1829208.000000, 9381, 0.098022
1849539.000000, 9485, 0.097802
1869878.000000, 9589, 0.095824
1890210.000000, 9694, 0.094725
1910530.000000, 9798, 0.098901
1930858.000000, 9902, 0.101978
1951158.000000, 10006, 0.094725
1971448.000000, 10111, 0.093846
1991767.000000, 10215, 0.095604
2012087.000000, 10319, 0.097143
```
9.43V

```asm
6.000000, 0, 0.094945
20627.000000, 66, 0.091648
40997.000000, 131, 0.092967
61285.000000, 196, 0.095385
81606.000000, 261, 0.092308
101925.000000, 326, 0.092967
122246.000000, 391, 0.092308
142579.000000, 457, 0.092308
163016.000000, 522, 0.090549
183307.000000, 587, 0.095824
203634.000000, 652, 0.095165
223935.000000, 717, 0.094286
244226.000000, 782, 0.094286
264546.000000, 847, 0.094066
284877.000000, 912, 0.097582
305198.000000, 977, 0.093407
325526.000000, 1043, 0.094066
345845.000000, 1108, 0.094505
366180.000000, 1173, 0.086593
386518.000000, 1238, 0.091648
406805.000000, 1303, 0.091868
427234.000000, 1368, 0.094725
447526.000000, 1433, 0.092967
467846.000000, 1498, 0.092967
488167.000000, 1563, 0.093626
508486.000000, 1628, 0.093407
528806.000000, 1693, 0.093626
549127.000000, 1759, 0.090330
569446.000000, 1824, 0.094066
589767.000000, 1889, 0.093626
610085.000000, 1954, 0.092088
630437.000000, 2019, 0.091868
650767.000000, 2084, 0.092967
671086.000000, 2149, 0.090989
691418.000000, 2214, 0.091209
711757.000000, 2279, 0.089011
732113.000000, 2345, 0.095604
752408.000000, 2410, 0.094286
772736.000000, 2475, 0.093846
793036.000000, 2540, 0.096264
813326.000000, 2605, 0.094286
833645.000000, 2670, 0.094945
853966.000000, 2735, 0.093187
874285.000000, 2800, 0.095165
894616.000000, 2865, 0.100000
914995.000000, 2930, 0.094945
935287.000000, 2995, 0.093407
955634.000000, 3060, 0.092967
975953.000000, 3126, 0.090549
996243.000000, 3191, 0.095604
1016563.000000, 3256, 0.091209
1036886.000000, 3321, 0.092967
1057205.000000, 3386, 0.093626
1077523.000000, 3451, 0.089890
1097845.000000, 3516, 0.095385
1118163.000000, 3581, 0.092747
1138498.000000, 3646, 0.088132
1158876.000000, 3711, 0.093407
1179404.000000, 3777, 0.100440
1199726.000000, 3843, 0.095604
1220046.000000, 3908, 0.092308
1240366.000000, 3973, 0.091209
1260686.000000, 4038, 0.095165
1281003.000000, 4103, 0.093626
1301325.000000, 4168, 0.091209
1321646.000000, 4233, 0.094505
1341967.000000, 4299, 0.096044
1362286.000000, 4364, 0.094286
1382605.000000, 4429, 0.095824
1402957.000000, 4494, 0.091868
1423247.000000, 4559, 0.095385
1443567.000000, 4624, 0.093407
1463887.000000, 4690, 0.090110
1484206.000000, 4755, 0.095604
1504525.000000, 4820, 0.093407
1524846.000000, 4885, 0.092527
1545166.000000, 4950, 0.092747
1565482.000000, 5015, 0.096703
1585767.000000, 5080, 0.092967
1606085.000000, 5146, 0.093846
1626406.000000, 5211, 0.095385
1646725.000000, 5276, 0.093626
1667084.000000, 5341, 0.093846
1687435.000000, 5407, 0.092527
1707754.000000, 5472, 0.094945
1728088.000000, 5537, 0.092308
1748408.000000, 5602, 0.096484
1768736.000000, 5667, 0.095385
1789037.000000, 5732, 0.091429
1809326.000000, 5798, 0.100440
1829656.000000, 5863, 0.092088
1849946.000000, 5928, 0.090549
1870277.000000, 5993, 0.088352
1890565.000000, 6058, 0.097143
1910885.000000, 6124, 0.093187
1931247.000000, 6189, 0.093626
1951594.000000, 6254, 0.093846
1971885.000000, 6319, 0.094066
1992205.000000, 6384, 0.091429
2012525.000000, 6450, 0.093626
```
5.93V