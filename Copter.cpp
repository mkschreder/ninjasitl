/*
	Copyright (c) 2016 Martin Schröder <mkschreder.uk@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Application.h"
#include "Copter.h"
 
/* set this always to the sampling in degrees for the table below */
#define SAMPLING_RES		10.0f
#define SAMPLING_MIN_LAT	-60.0f
#define SAMPLING_MAX_LAT	60.0f
#define SAMPLING_MIN_LON	-180.0f
#define SAMPLING_MAX_LON	180.0f

/* table data containing magnetic declination angle in degrees */
const float declination_table[13][37] =
{
	{47.225,46.016,44.6,43.214,41.929,40.625,38.996,36.648,33.246,28.649,22.973,16.599,10.096,4.0412,-1.2113,-5.7129,-9.9239,-14.505,-19.992,-26.538,-33.881,-41.531,-48.995,-55.908,-62.043,-67.26,-71.433,-74.32,-75.32,-72.759,-61.089,-22.66,24.673,41.679,46.715,47.77,47.225},
	{30.663,30.935,30.729,30.365,30.083,29.989,29.903,29.306,27.493,23.864,18.229,11.012,3.2503,-3.7771,-9.1638,-12.793,-15.299,-17.753,-21.275,-26.491,-33.098,-40.129,-46.619,-51.911,-55.576,-57.256,-56.482,-52.464,-44.092,-30.765,-14.236,1.4682,13.543,21.705,26.727,29.465,30.663},
	{22.141,22.67,22.786,22.665,22.457,22.365,22.478,22.512,21.659,18.814,13.22,5.1441,-3.8724,-11.749,-17.208,-20.303,-21.777,-22.354,-23.04,-25.422,-30.143,-35.947,-41.114,-44.536,-45.603,-43.943,-39.28,-31.541,-21.622,-11.631,-3.1023,3.9413,9.8774,14.757,18.445,20.858,22.141},
	{16.701,17.175,17.415,17.473,17.283,16.943,16.689,16.566,15.935,13.436,7.8847,-0.52637,-9.7982,-17.399,-22.182,-24.63,-25.544,-24.89,-22.583,-20.489,-21.371,-25.128,-29.294,-31.76,-31.569,-28.686,-23.466,-16.487,-9.2421,-3.5196,0.52944,4.0043,7.5112,10.847,13.654,15.633,16.701},
	{13.062,13.307,13.488,13.647,13.552,13.138,12.644,12.281,11.511,8.9083,3.288,-4.9363,-13.534,-20.108,-23.761,-24.943,-24.267,-21.633,-16.975,-11.895,-9.3052,-10.608,-14.22,-17.235,-17.896,-16.21,-12.734,-7.944,-3.1917,-0.09448,1.5925,3.3685,5.7977,8.374,10.643,12.277,13.062},
	{10.819,10.774,10.744,10.873,10.857,10.513,10.051,9.6504,8.6623,5.7203,-0.00103,-7.7113,-15.239,-20.543,-22.747,-22.021,-19.117,-14.884,-10.06,-5.4668,-2.2918,-1.8711,-4.2049,-7.1881,-8.7089,-8.4387,-6.7419,-3.8048,-0.7327,0.90936,1.399,2.3853,4.3796,6.6682,8.7425,10.241,10.819},
	{9.6261,9.4267,9.2035,9.2862,9.3524,9.1115,8.7204,8.235,6.8708,3.4733,-2.2704,-9.2885,-15.646,-19.575,-20.227,-17.864,-13.663,-9.1469,-5.2757,-2.071,0.49345,1.5251,0.29098,-2.0644,-3.6913,-4.1179,-3.5596,-1.9862,-0.1684,0.55548,0.42539,1.019,2.8249,5.1073,7.319,8.9998,9.6261},
	{8.9369,8.9745,8.8157,9.0087,9.2615,9.1408,8.6708,7.7842,5.7199,1.6936,-4.0873,-10.373,-15.479,-17.961,-17.333,-14.207,-9.8358,-5.6345,-2.4989,-0.22009,1.7216,2.8156,2.1698,0.36849,-1.0742,-1.6949,-1.7318,-1.1504,-0.42435,-0.50061,-1.1161,-0.88856,0.69851,3.0638,5.6417,7.8392,8.9369},
	{8.0661,8.9128,9.3097,9.8875,10.445,10.466,9.7864,8.2288,5.1818,0.28672,-5.7524,-11.438,-15.264,-16.348,-14.841,-11.596,-7.5597,-3.7288,-0.91243,0.96946,2.51,3.5079,3.1926,1.8339,0.62026,-0.01417,-0.36046,-0.53631,-0.84781,-1.8165,-3.0616,-3.3677,-2.1514,0.22612,3.1996,6.0876,8.0661},
	{6.6045,8.7101,10.209,11.491,12.418,12.553,11.622,9.3409,5.2254,-0.73322,-7.3408,-12.717,-15.505,-15.494,-13.422,-10.206,-6.4719,-2.8539,-0.05159,1.8185,3.2182,4.1923,4.2452,3.4363,2.5284,1.8671,1.1902,0.25353,-1.1684,-3.2141,-5.2908,-6.2436,-5.4483,-3.1334,0.11904,3.6145,6.6045},
	{4.8686,8.2538,11.062,13.264,14.651,14.898,13.714,10.725,5.4958,-1.7139,-9.1709,-14.568,-16.765,-16.116,-13.666,-10.317,-6.5773,-2.906,0.14042,2.3482,3.9956,5.2703,5.9678,5.9805,5.5419,4.7751,3.4951,1.4836,-1.3442,-4.7282,-7.762,-9.283,-8.7476,-6.4234,-2.9491,1.0086,4.8686},
	{3.5965,7.9367,11.797,14.893,16.888,17.409,16.035,12.22,5.5033,-3.468,-12.144,-17.817,-19.749,-18.751,-15.97,-12.268,-8.1724,-4.1017,-0.46224,2.551,5.0589,7.2268,8.9817,10.082,10.298,9.416,7.21,3.5705,-1.2522,-6.3856,-10.448,-12.313,-11.73,-9.1743,-5.367,-0.9513,3.5965},
	{3.0633,8.0696,12.667,16.527,19.239,20.252,18.783,13.765,4.4393,-7.6723,-18.2,-24.093,-25.554,-23.984,-20.61,-16.25,-11.425,-6.5023,-1.759,2.6571,6.7294,10.458,13.708,16.148,17.3,16.587,13.441,7.6143,-0.14961,-7.7957,-13.103,-15.143,-14.253,-11.262,-6.9868,-2.0631,3.0633}
};

/* table data containing magnetic inclination angle in degrees */
const float inclination_table[13][37] =
{
	{-77.62,-75.621,-73.703,-71.805,-69.84,-67.703,-65.317,-62.69,-59.968,-57.429,-55.437,-54.313,-54.191,-54.921,-56.118,-57.329,-58.214,-58.643,-58.712,-58.692,-58.934,-59.74,-61.257,-63.461,-66.211,-69.329,-72.657,-76.065,-79.441,-82.661,-85.506,-87.208,-86.36,-84.237,-81.951,-79.73,-77.62},
	{-71.644,-69.722,-67.862,-66.038,-64.191,-62.202,-59.91,-57.194,-54.113,-51.035,-48.639,-47.697,-48.625,-51.137,-54.39,-57.488,-59.821,-61.085,-61.237,-60.574,-59.768,-59.618,-60.616,-62.761,-65.711,-69.044,-72.416,-75.556,-78.175,-79.936,-80.593,-80.201,-79.054,-77.45,-75.591,-73.62,-71.644},
	{-64.379,-62.435,-60.505,-58.564,-56.621,-54.665,-52.574,-50.107,-47.083,-43.756,-41.093,-40.479,-42.721,-47.229,-52.537,-57.471,-61.513,-64.379,-65.669,-65.164,-63.395,-61.672,-61.269,-62.55,-65.012,-67.881,-70.533,-72.526,-73.547,-73.61,-73.075,-72.251,-71.191,-69.844,-68.201,-66.334,-64.379},
	{-54.946,-52.868,-50.792,-48.616,-46.372,-44.193,-42.1,-39.8,-36.829,-33.264,-30.385,-30.266,-34.046,-40.628,-47.854,-54.358,-59.856,-64.283,-67.119,-67.68,-65.968,-63.1,-60.894,-60.535,-61.761,-63.584,-65.189,-66.021,-65.775,-64.769,-63.698,-62.826,-61.898,-60.654,-59.023,-57.056,-54.946},
	{-42.121,-39.754,-37.516,-35.153,-32.626,-30.14,-27.828,-25.343,-22.028,-18.017,-15.062,-15.744,-21.265,-30.112,-39.56,-47.788,-54.306,-59.187,-62.247,-63.04,-61.444,-58.163,-54.854,-53.154,-53.207,-54.092,-55.004,-55.267,-54.381,-52.801,-51.562,-50.874,-50.1,-48.81,-46.97,-44.643,-42.121},
	{-25.138,-22.301,-19.912,-17.531,-14.951,-12.37,-9.9146,-7.1192,-3.3525,0.86465,3.4285,1.8632,-4.8485,-15.374,-26.858,-36.666,-43.599,-47.78,-49.783,-49.893,-48.024,-44.445,-40.638,-38.414,-38.009,-38.508,-39.247,-39.498,-38.5,-36.768,-35.692,-35.432,-34.911,-33.532,-31.323,-28.369,-25.138},
	{-4.9962,-1.7126,0.71401,2.8847,5.219,7.5873,9.91,12.689,16.306,19.897,21.553,19.47,12.842,2.3904,-9.4406,-19.56,-26.119,-29.178,-29.919,-29.286,-27.194,-23.471,-19.459,-17.133,-16.698,-17.132,-17.899,-18.371,-17.66,-16.255,-15.708,-16.114,-16.082,-14.846,-12.422,-8.9145,-4.9962},
	{14.869,18.185,20.462,22.293,24.256,26.334,28.442,30.896,33.762,36.172,36.784,34.538,28.924,20.356,10.616,2.2154,-3.0793,-5.0649,-4.8917,-3.7875,-1.7639,1.5757,5.1722,7.2324,7.5774,7.2059,6.5718,6.0595,6.3467,7.0334,6.8289,5.7043,4.9959,5.5796,7.5845,10.93,14.869},
	{31.164,34.006,36.041,37.647,39.383,41.332,43.382,45.569,47.716,49.104,48.9,46.613,42.181,36.116,29.611,24.084,20.599,19.485,20.056,21.274,22.956,25.416,28.017,29.529,29.786,29.536,29.184,28.895,28.929,28.95,28.145,26.531,25.104,24.717,25.695,28.042,31.164},
	{43.422,45.479,47.235,48.815,50.558,52.55,54.665,56.761,58.528,59.374,58.753,56.548,53.102,49.074,45.208,42.092,40.16,39.643,40.248,41.324,42.606,44.197,45.811,46.815,47.078,47.021,46.948,46.916,46.893,46.555,45.437,43.596,41.72,40.533,40.438,41.516,43.422},
	{53.102,54.356,55.814,57.449,59.331,61.427,63.597,65.639,67.216,67.832,67.107,65.133,62.449,59.699,57.346,55.602,54.591,54.397,54.884,55.703,56.613,57.588,58.539,59.25,59.652,59.904,60.137,60.33,60.32,59.812,58.553,56.66,54.639,53.044,52.229,52.294,53.102},
	{61.877,62.601,63.785,65.363,67.255,69.332,71.425,73.313,74.675,75.099,74.352,72.651,70.529,68.494,66.843,65.687,65.049,64.919,65.198,65.705,66.286,66.893,67.525,68.165,68.802,69.44,70.041,70.469,70.5,69.901,68.591,66.775,64.856,63.228,62.144,61.699,61.877},
	{70.565,71.043,71.969,73.282,74.89,76.669,78.458,80.035,81.082,81.248,80.439,78.963,77.256,75.651,74.334,73.378,72.792,72.544,72.57,72.785,73.123,73.565,74.13,74.845,75.713,76.683,77.618,78.289,78.419,77.836,76.61,75.025,73.414,72.04,71.067,70.568,70.565}
};

/* table data containing magnetic intensity in Gauss */
const float intensity_table[13][37] =
{
	{0.62195,0.60352,0.58407,0.56378,0.54235,0.5192,0.49375,0.46596,0.43661,0.40734,0.38017,0.35689,0.33841,0.32458,0.31447,0.30707,0.302,0.29983,0.30197,0.31023,0.32606,0.35005,0.38167,0.41941,0.4611,0.50431,0.54648,0.58515,0.61805,0.64341,0.66031,0.66875,0.6695,0.66384,0.65315,0.6388,0.62195},
	{0.58699,0.56471,0.54222,0.51977,0.49699,0.47285,0.44606,0.4159,0.38299,0.3495,0.31875,0.29401,0.2771,0.26745,0.26251,0.25941,0.25657,0.25431,0.25469,0.26104,0.27676,0.30379,0.34157,0.38741,0.43751,0.48812,0.53602,0.57843,0.61292,0.63771,0.65218,0.65693,0.65331,0.64296,0.62744,0.6083,0.58699},
	{0.54115,0.51716,0.49337,0.47003,0.44701,0.42359,0.39839,0.37018,0.33882,0.30605,0.27573,0.25265,0.23995,0.23673,0.23869,0.24146,0.24284,0.24258,0.24188,0.24416,0.25504,0.27952,0.31847,0.36815,0.42238,0.47534,0.52306,0.56286,0.59276,0.61199,0.62133,0.62218,0.61584,0.60338,0.58585,0.56455,0.54115},
	{0.48901,0.46546,0.44226,0.41941,0.39712,0.37544,0.35375,0.33083,0.30548,0.27808,0.25206,0.23294,0.22466,0.22623,0.23284,0.24043,0.24771,0.25392,0.25756,0.2592,0.26416,0.28046,0.31301,0.35947,0.41201,0.4626,0.5061,0.53933,0.56053,0.57088,0.5738,0.57145,0.56393,0.55115,0.53361,0.51231,0.48901},
	{0.43275,0.41204,0.39186,0.37191,0.35253,0.33441,0.31775,0.3017,0.28442,0.26511,0.24603,0.23176,0.22605,0.22892,0.2372,0.24818,0.2613,0.27524,0.28613,0.2912,0.29315,0.29999,0.32028,0.35581,0.39969,0.44294,0.47952,0.50496,0.51672,0.51792,0.51496,0.51028,0.50215,0.48957,0.47314,0.45367,0.43275},
	{0.37936,0.36387,0.34916,0.33493,0.32156,0.30968,0.29963,0.29094,0.28194,0.27119,0.25923,0.24862,0.24246,0.2429,0.25003,0.26227,0.27786,0.29465,0.30882,0.3168,0.31873,0.3201,0.32969,0.35219,0.38333,0.41554,0.44325,0.46129,0.46614,0.4614,0.45461,0.44801,0.43895,0.4265,0.41178,0.39569,0.37936},
	{0.34151,0.33271,0.3248,0.31786,0.31251,0.30889,0.30674,0.30563,0.30411,0.30005,0.29252,0.28275,0.27347,0.26836,0.27051,0.27963,0.29265,0.30666,0.31919,0.32782,0.33164,0.33341,0.33926,0.35308,0.37271,0.39367,0.41222,0.42402,0.42574,0.41956,0.41085,0.40143,0.39009,0.37701,0.36387,0.35184,0.34151},
	{0.32861,0.32598,0.32444,0.32451,0.32723,0.33225,0.33844,0.34476,0.34925,0.34892,0.34232,0.33071,0.31718,0.30603,0.30142,0.30408,0.3115,0.32118,0.33137,0.34024,0.34686,0.35275,0.36068,0.37146,0.38401,0.39715,0.40907,0.41682,0.418,0.41256,0.40201,0.38791,0.37189,0.35616,0.34293,0.33373,0.32861},
	{0.34027,0.34127,0.34464,0.3507,0.36038,0.37294,0.3865,0.39917,0.40815,0.4099,0.40274,0.38831,0.37072,0.3552,0.34582,0.34316,0.34566,0.35218,0.36135,0.37089,0.37971,0.38903,0.39965,0.41036,0.42049,0.43072,0.44044,0.44743,0.44945,0.44455,0.43158,0.41195,0.38966,0.369,0.35302,0.34348,0.34027},
	{0.37253,0.3743,0.38082,0.39172,0.4068,0.42456,0.44274,0.45904,0.47046,0.47349,0.4662,0.45016,0.43012,0.41204,0.39986,0.39392,0.39314,0.39717,0.40505,0.41434,0.42357,0.43356,0.44488,0.45654,0.46802,0.47977,0.49115,0.5,0.50357,0.49896,0.48426,0.46097,0.43398,0.40879,0.38917,0.37703,0.37253},
	{0.42232,0.42361,0.43153,0.44511,0.46277,0.48222,0.50102,0.51697,0.52763,0.53021,0.5231,0.50749,0.48759,0.46872,0.4545,0.44574,0.44202,0.44314,0.44829,0.45556,0.46359,0.47278,0.48398,0.49721,0.51197,0.52753,0.54226,0.5536,0.55855,0.55434,0.53968,0.51634,0.489,0.46299,0.44215,0.4284,0.42232},
	{0.48297,0.48409,0.49144,0.50386,0.51947,0.53592,0.55104,0.56298,0.57002,0.57048,0.5635,0.54997,0.53268,0.51524,0.50048,0.48973,0.48333,0.48125,0.48294,0.48735,0.49374,0.50234,0.51385,0.52856,0.54583,0.56407,0.58089,0.59345,0.59903,0.5957,0.58321,0.56356,0.54058,0.51853,0.50058,0.4885,0.48297},
	{0.53888,0.53954,0.54416,0.55185,0.5613,0.57099,0.5795,0.58558,0.58818,0.58651,0.58029,0.57006,0.55723,0.54365,0.53109,0.52078,0.51346,0.50946,0.50872,0.51103,0.51624,0.52445,0.53585,0.55027,0.56682,0.58384,0.59914,0.61041,0.61575,0.61419,0.60607,0.59307,0.5778,0.563,0.55086,0.54264,0.53888}
};

/*
 calculate magnetic field intensity and orientation
*/
bool get_mag_field_ef(float latitude_deg, float longitude_deg, float &intensity_gauss, float &declination_deg, float &inclination_deg){
    bool valid_input_data = true;

    /* round down to nearest sampling resolution */
    int min_lat = (int)(latitude_deg / SAMPLING_RES) * SAMPLING_RES;
    int min_lon = (int)(longitude_deg / SAMPLING_RES) * SAMPLING_RES;

    /* for the rare case of hitting the bounds exactly
     * the rounding logic wouldn't fit, so enforce it.
     */

    /* limit to table bounds - required for maxima even when table spans full globe range */
    if (latitude_deg <= SAMPLING_MIN_LAT) {
        min_lat = SAMPLING_MIN_LAT;
        valid_input_data = false;
    }

    if (latitude_deg >= SAMPLING_MAX_LAT) {
        min_lat = (int)(latitude_deg / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES;
        valid_input_data = false;
    }

    if (longitude_deg <= SAMPLING_MIN_LON) {
        min_lon = SAMPLING_MIN_LON;
        valid_input_data = false;
    }

    if (longitude_deg >= SAMPLING_MAX_LON) {
        min_lon = (int)(longitude_deg / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES;
        valid_input_data = false;
    }

    /* find index of nearest low sampling point */
    unsigned min_lat_index = (-(SAMPLING_MIN_LAT) + min_lat)  / SAMPLING_RES;
    unsigned min_lon_index = (-(SAMPLING_MIN_LON) + min_lon) / SAMPLING_RES;

    /* calculate intensity */

    float data_sw = intensity_table[min_lat_index][min_lon_index];
    float data_se = intensity_table[min_lat_index][min_lon_index + 1];;
    float data_ne = intensity_table[min_lat_index + 1][min_lon_index + 1];
    float data_nw = intensity_table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */

    float data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    float data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;

    intensity_gauss = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;

    /* calculate declination */

    data_sw = declination_table[min_lat_index][min_lon_index];
    data_se = declination_table[min_lat_index][min_lon_index + 1];;
    data_ne = declination_table[min_lat_index + 1][min_lon_index + 1];
    data_nw = declination_table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */

    data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;

    declination_deg = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;

    /* calculate inclination */

    data_sw = inclination_table[min_lat_index][min_lon_index];
    data_se = inclination_table[min_lat_index][min_lon_index + 1];;
    data_ne = inclination_table[min_lat_index + 1][min_lon_index + 1];
    data_nw = inclination_table[min_lat_index + 1][min_lon_index];

    /* perform bilinear interpolation on the four grid corners */

    data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw;
    data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw;

    inclination_deg = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min;

    return valid_input_data;

}

Copter::Copter(Application *app){
	float mass = 2.0f; 
	mApp = app;

	// set home location
	_home = glm::ivec3(149.165230 * 1.0e7, 584 * 1e2, -35.363261 * 1.0e7); 

	// Create a box rigid body
	ISceneManager *irrScene = mApp->irrScene;
	
	IAnimatedMesh *mesh = irrScene->getMesh("quad_body.obj"); 
	ISceneNode *Node = irrScene->addAnimatedMeshSceneNode(mesh);

	//Node->setScale(vector3df(1, 0.1, 1));
	//Node->setMaterialFlag(EMF_LIGHTING, 1);
	//Node->setMaterialFlag(EMF_NORMALIZE_NORMALS, true);
	Node->setMaterialTexture(0, mApp->_drv->getTexture("rust0.jpg"));
	mNode = Node;

	// add motor cubes	
	/*
	glm::mat4 pmat(
		glm::vec4(0.0, 0.05, 0.5, 0.0),
		glm::vec4(0.5, 0.05, 0.0, 0.0),
		glm::vec4(0.0, 0.05, -0.5, 0.0),
		glm::vec4(-0.5,0.05, 0.0, 0.0));
	for(int c = 0; c < 4; c++){
		glm::vec4 pos = pmat[c];
		ISceneNode *pn = irrScene->addCubeSceneNode(1.0f);
		pn->setPosition(vector3df(pos.x, pos.y, pos.z)); 
		pn->setScale(vector3df(0.5, 1.0, 0.5));
		pn->setMaterialFlag(EMF_LIGHTING, 1);
		pn->setMaterialFlag(EMF_NORMALIZE_NORMALS, true);
		pn->setMaterialTexture(0, mApp->_drv->getTexture("rust0.jpg"));
		Node->addChild(pn); 
	}
	*/
	// Set the initial position of the object
	btTransform Transform;
	Transform.setIdentity();
	Transform.setOrigin(btVector3(0, 0.5, 0));

	btDefaultMotionState *MotionState = new btDefaultMotionState(Transform);

	// Create the shape
	btVector3 HalfExtents(0.5f, 0.05f, 0.5f);
	btCollisionShape *Shape = new btBoxShape(HalfExtents);
	
	// Add mass
	btVector3 LocalInertia;
	Shape->calculateLocalInertia(mass, LocalInertia);

	// Create the rigid body object
	btRigidBody::btRigidBodyConstructionInfo info(mass, MotionState, Shape, LocalInertia); //motion state would actually be non-null in most real usages
	info.m_restitution = 2.0f;
	info.m_friction = 1.5f;

	mBody = new btRigidBody(info);
	//mBody->setActivationState(DISABLE_DEACTIVATION);
	//mBody->forceActivationState(DISABLE_SIMULATION);
	
	//_simulate = true; 

	// Store a pointer to the irrlicht node so we can update it later
	mBody->setUserPointer((void *)(Node));

	mBody->setDamping(0.05, 0.1); 

	// Add it to the world
	mApp->World->addRigidBody(mBody);
	mApp->Objects.push_back(mBody);
/*
	_motors[1-1] = (struct motor){ glm::vec3( 0.5, 0.05, 0.0), MOTOR_CCW };
	_motors[2-1] = (struct motor){ glm::vec3(-0.5, 0.05, 0.0), MOTOR_CCW };
	_motors[3-1] = (struct motor){ glm::vec3( 0.0, 0.05, 0.5), MOTOR_CW };
	_motors[4-1] = (struct motor){ glm::vec3( 0.0, 0.05,-0.5), MOTOR_CW };
*/
	_motors[1-1] = (struct motor){ glm::vec3( 0.15, 0.0, 0.1), MOTOR_CCW };
	_motors[2-1] = (struct motor){ glm::vec3(-0.15, 0.0,-0.1), MOTOR_CCW };
	_motors[3-1] = (struct motor){ glm::vec3(-0.15, 0.0, 0.1), MOTOR_CW };
	_motors[4-1] = (struct motor){ glm::vec3( 0.15, 0.0,-0.1), MOTOR_CW };

	for(int c = 0; c < 4; c++){
		_motors[c].thrust = 0.2; 
	}
	
	//FlightController::setSensorProvider(new Copter(mApp->World, RigidBody));
}

void Copter::setSimulationOn(bool on){
	mApp->World->removeRigidBody(mBody);
	if(on){
		mBody->setActivationState(DISABLE_DEACTIVATION);
		mBody->setActivationState(ACTIVE_TAG);
		//mBody->setDamping(0.5, 0.8); 
	} else {	
		mBody->setActivationState(DISABLE_SIMULATION);
		//mBody->setDamping(0, 0); 
	}
	mApp->World->addRigidBody(mBody);
	_simulate = on; 
}

void Copter::setOutputThrust(unsigned int id, float value){
	if(id > (sizeof(_motors) / sizeof(_motors[0]))) return; 
	if(value < 0) value = 0; if(value > 1.0) value = 1.0; 
	_motors[id].thrust = value; 
	if(_motors[id].dir == MOTOR_CW)
		_motors[id].torque = glm::cross(glm::normalize(_motors[id].pos), glm::vec3(0, 1.0f, 0)); 
	else if(_motors[id].dir == MOTOR_CCW)
		_motors[id].torque = glm::cross(glm::normalize(_motors[id].pos), glm::vec3(0, -1.0f, 0)); 
}

void Copter::updateLocation(){
	glm::vec3 position = getPosition(); 

	// scaling factor from 1e-7 degrees to meters at equater
	// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
	#define LOCATION_SCALING_FACTOR 0.011131884502145034f
	// inverse of LOCATION_SCALING_FACTOR
	#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f

	float lat = _home.z; 
	float lng = _home.x; 
	float scale = cosf(glm::radians(lat * 1.0e-7f));
	if(scale < 0.01f) scale = 0.01f; else if(scale > 1.0f) scale = 1.0f; 

	int32_t dlat = position.z * LOCATION_SCALING_FACTOR_INV;
	int32_t dlng = (position.x * LOCATION_SCALING_FACTOR_INV) / scale;

	_location.x = lng + dlng;
	_location.y = _home.y + position.y * 100.0f; 
	_location.z = lat + dlat;
}

glm::vec3 Copter::calcAcceleration(){
	return glm::inverse(getRotation()) * (_linear_acceleration + glm::vec3(0, 9.82, 0)); 
}

glm::vec3 Copter::calcMagFieldIntensity(){
	glm::quat rot = getRotation(); 

	// get the magnetic field intensity and orientation
	float intensity;
	float declination;                  
	float inclination;
	get_mag_field_ef(_location.z*1e-7f,_location.x*1e-7f,intensity,declination,inclination);

	// create a field vector and rotate to the required orientation
	glm::vec3 mag_ef(0, 0, 1e3f * intensity); 
	glm::quat mrot = 
		glm::angleAxis(glm::radians(declination), glm::vec3(0, 1, 0)) *    
		glm::angleAxis(glm::radians(inclination), glm::vec3(1, 0, 0));  

			//glm::quat(cos(-glm::radians(declination/2.0f)), 0, sin(-glm::radians(declination/2.0f)), 0) * 
			//glm::quat(cos(glm::radians(inclination/2.0f)), sin(glm::radians(inclination/2.0f)), 0, 0); 
	//R.from_euler(0, -ToRad(inclination), ToRad(declination));
	return glm::inverse(rot) * (mrot * mag_ef);                

	/*
	// calculate frame height above ground   
	float frame_height_agl = fmaxf(position.y, 0.0f);
	  
	// calculate scaling factor that varies from 1 at ground level to 1/8 at sitl->mag_anomaly_hgt
	// Assume magnetic anomaly strength scales with 1/R**3
	float mag_anomaly_hgt = 0.0; 
	glm::vec3 mag_anomaly_ned(0.0, 0.0, 0.0); 
	float anomaly_scaler = (mag_anomaly_hgt / (frame_height_agl + mag_anomaly_hgt));
	anomaly_scaler = anomaly_scaler * anomaly_scaler * anomaly_scaler;
	  
	// add scaled anomaly to earth field
	mag_ef += mag_anomaly_ned * anomaly_scaler;
	 */ 
	//_mag = mag_ef; 
	  
	//::printf("mag: %f %f %f\n", _mag.x, _mag.y, _mag.z); 
	// add motor interference           
	//_mag += mag_mot * battery_current;
}

void Copter::render(irr::video::IVideoDriver *drv){
	glm::quat rot = getRotation();
	glm::vec3 pos = getPosition(); // + rot * glm::vec3(0, 0.2, 0);
	glm::vec3 acc = rot * _accel;  
	glm::vec3 mag = (rot * _mag) * 0.001f;  
	glm::vec3 vel = getVelocity(); 
	glm::vec3 ax = rot * glm::vec3(0.1, 0.0, 0.0); 
	glm::vec3 ay = rot * glm::vec3(0.0, 0.1, 0.0); 
	glm::vec3 az = rot * glm::vec3(0.0, 0.0, 0.1); 

	drv->setTransform(irr::video::ETS_WORLD, irr::core::IdentityMatrix);

	SMaterial m;
	m.Lighting = false;
	drv->setMaterial(m);
	
	glm::vec3 hit, norm; 
	/*
	if(mApp->clipRay(pos, pos + rot * glm::vec3(100.0, 0, 0.0), &hit, &norm)){
		drv->draw3DBox(aabbox3df(vector3df(hit.x - 0.1, hit.y - 0.1, hit.z - 0.1), vector3df(hit.x + 0.1, hit.y + 0.1, hit.z + 0.1)), SColor(255, 255, 0, 0)); 
		drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
			vector3df(hit.x, hit.y, hit.z), 
			SColor( 255, 255, 0, 0 ));
	}*/

	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + ax.x, pos.y + ax.y, pos.z + ax.z), 
		SColor( 255, 255, 0, 0 ));
	
	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + ay.x, pos.y + ay.y, pos.z + ay.z), 
		SColor( 255, 0, 255, 0 ));

	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + az.x, pos.y + az.y, pos.z + az.z), 
		SColor( 255, 0, 0, 255 ));

	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + acc.x, pos.y + acc.y, pos.z + acc.z), 
		SColor( 255, 255, 255, 255 ));

	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + mag.x, pos.y + mag.y, pos.z + mag.z), 
		SColor( 255, 255, 255, 255 ));

	glm::vec3 cmag = rot * calcMagFieldIntensity() * 0.001f; 
	glm::vec3 cacc = rot * calcAcceleration(); 
	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + cmag.x, pos.y + cmag.y, pos.z + cmag.z), 
		SColor( 255, 255, 255, 0 ));

	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + cacc.x, pos.y + cacc.y, pos.z + cacc.z), 
		SColor( 255, 255, 255, 0 ));

	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + _linear_acceleration.x, pos.y + _linear_acceleration.y, pos.z + _linear_acceleration.z), 
		SColor( 255, 255, 0, 255 ));

	drv->draw3DLine(vector3df(pos.x, pos.y, pos.z),
		vector3df(pos.x + vel.x, pos.y + vel.y, pos.z + vel.z), 
		SColor( 255, 255, 0, 255 ));


	for(int c = 0; c < 4; c++){
		float th = _motors[c].thrust; 
		glm::vec3 p = rot * _motors[c].pos + pos; 
		glm::vec3 f = rot * glm::vec3(0, 1.0, 0) * th; 
		glm::vec3 a = rot * _motors[c].torque * th * 0.4f; 
		
		// draw direction of thrust
		drv->draw3DLine(vector3df(p.x, p.y, p.z),
			vector3df(p.x + f.x, p.y + f.y, p.z + f.z), SColor(255, 255, 255, 0));

		// draw direction of torque 
		drv->draw3DLine(vector3df(p.x, p.y, p.z),
			vector3df(p.x + a.x, p.y + a.y, p.z + a.z), SColor(255, 255, 0, 0));

	}	
}

glm::vec3 Copter::getPosition(){
	btVector3 pos = mBody->getCenterOfMassPosition();
	return glm::vec3(pos[0], pos[1], pos[2]); 
}

glm::ivec3 Copter::getLocation(){
	return _location; 
}

glm::vec3 Copter::getMagneticField(){
	return _mag; 
}

glm::quat Copter::getRotation(){
	const btQuaternion& quat = mBody->getOrientation();
	return glm::quat(quat.w(), quat.x(), quat.y(), quat.z()); 
}

void Copter::setPosition(const glm::vec3 &pos){
	btTransform t = mBody->getCenterOfMassTransform();
	t.setOrigin(btVector3(pos.x, pos.y, pos.z)); 
	mBody->setCenterOfMassTransform(t); 
	mNode->setPosition(vector3df(pos.x, pos.y, pos.z)); 
}

void Copter::setRotation(const glm::quat &rot){
	btTransform t = mBody->getCenterOfMassTransform();
	btQuaternion q(rot.x, rot.y, rot.z, rot.w); 
	t.setRotation(q); 
	mBody->setCenterOfMassTransform(t); 
}

void Copter::setAngularVelocity(const glm::vec3 &v){
	mBody->setAngularVelocity(btVector3(v.x, v.y, v.z)); 
}

void Copter::setLinearVelocity(const glm::vec3 &v){
	if(_simulate)
		mBody->setLinearVelocity(btVector3(v.x, v.y, v.z)); 
	else
		_velocity = v; 
}

void Copter::setAccelerometer(const glm::vec3 &v){
	_accel = v; 
}

void Copter::setMagneticField(const glm::vec3 &v){
	_mag = v; 
}

glm::vec3 Copter::getAccel(){
	return _accel; 
}

glm::vec3 Copter::getGyro(){
	glm::quat rot = getRotation(); 
	glm::vec3 lx = rot * glm::vec3(1, 0, 0); 
	glm::vec3 ly = rot * glm::vec3(0, 1, 0); 
	glm::vec3 lz = rot * glm::vec3(0, 0, 1); 

	btVector3 av = mBody->getAngularVelocity(); 

	float xr = lx.x * av[0] + lx.y * av[1] + lx.z * av[2]; 
	float yr = ly.x * av[0] + ly.y * av[1] + ly.z * av[2]; 
	float zr = lz.x * av[0] + lz.y * av[1] + lz.z * av[2]; 
	
	//btVector3 tmp = mBody->getAngularVelocity(); 
	// pitch, yaw, roll
	return glm::vec3(xr, yr, zr);
}

glm::vec3 Copter::getVelocity(){
	btVector3 v = mBody->getLinearVelocity();
	return glm::vec3(v[0], v[1], v[2]); 
}

void Copter::update(float dt){
	glm::quat rot = getRotation(); 

	//btVector3 tot = mBody->getTotalForce(); 
	//printf("tot: %f %f %f\n", tot[0], tot[1], tot[2]); 
	// + frame

	//printf("thrust: "); 
	if(_simulate){
		for(int c = 0; c < 4; c++){
			float th = _motors[c].thrust * 10.0f; 
			//printf("%f ", th); 
			glm::vec3 p = rot * _motors[c].pos;
			glm::vec3 f = rot * glm::vec3(0, 1.0, 0) * th; 
			glm::vec3 a = rot * _motors[c].torque * th * 0.4f; 
			
			mBody->applyForce(btVector3(f.x, f.y, f.z), btVector3(p.x, p.y, p.z)); 
			mBody->applyForce(btVector3(a.x, a.y, a.z), btVector3(p.x, p.y, p.z)); 
		}

		updateLocation(); 

		// update accel
		glm::vec3 vel = getVelocity(); 
		_linear_acceleration = (vel - _velocity) / dt; // / 0.018f; 
		_velocity = vel; 

		_accel = calcAcceleration(); 
		_mag = calcMagFieldIntensity(); 
	} else {
		// update position based on velocity
		//setPosition(getPosition() + _velocity * dt); 
		updateLocation(); 
	}
	//printf("\n"); 
}
