function CodeMetrics() {
	 this.metricsArray = {};
	 this.metricsArray.var = new Array();
	 this.metricsArray.fcn = new Array();
	 this.metricsArray.fcn["atan2"] = {file: "/usr/local/MATLAB/R2021b/polyspace/verifier/cxx/include/include-libc/bits/mathcalls.h",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["crazyflie"] = {file: "/home/bober2/Desktop/school/SSY191/project2024-group22/generate_code/simulink-model/slprj/ert/crazyflie/crazyflie.c",
	stack: 56,
	stackTotal: 56};
	 this.metricsArray.fcn["sqrt"] = {file: "/usr/local/MATLAB/R2021b/polyspace/verifier/cxx/include/include-libc/bits/mathcalls.h",
	stack: 0,
	stackTotal: 0};
	 this.getMetrics = function(token) { 
		 var data;
		 data = this.metricsArray.var[token];
		 if (!data) {
			 data = this.metricsArray.fcn[token];
			 if (data) data.type = "fcn";
		 } else { 
			 data.type = "var";
		 }
	 return data; }; 
	 this.codeMetricsSummary = '<a href="javascript:void(0)" onclick="return postParentWindowMessage({message:\'gotoReportPage\', pageName:\'crazyflie_metrics\'});">Global Memory: 0(bytes) Maximum Stack: 56(bytes)</a>';
	}
CodeMetrics.instance = new CodeMetrics();
