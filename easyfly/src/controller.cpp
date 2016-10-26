,m_pidX(
		get(n, "PIDs/X/kp"),
		get(n, "PIDs/X/kd"),
		get(n, "PIDs/X/ki"),
		get(n, "PIDs/X/kpp"),
		get(n, "PIDs/X/minOutput"),
		get(n, "PIDs/X/maxOutput"),
		get(n, "PIDs/X/integratorMin"),
		get(n, "PIDs/X/integratorMax"),
		"x")
	,m_pidY(
		get(n, "PIDs/Y/kp"),
		get(n, "PIDs/Y/kd"),
		get(n, "PIDs/Y/ki"),
		get(n, "PIDs/Y/kpp"),
		get(n, "PIDs/Y/minOutput"),
		get(n, "PIDs/Y/maxOutput"),
		get(n, "PIDs/Y/integratorMin"),
		get(n, "PIDs/Y/integratorMax"),
		"y")
	,m_pidZ(
		get(n, "PIDs/Z/kp"),
		get(n, "PIDs/Z/kd"),
		get(n, "PIDs/Z/ki"),
		get(n, "PIDs/Z/kpp"),
		get(n, "PIDs/Z/minOutput"),
		get(n, "PIDs/Z/maxOutput"),
		get(n, "PIDs/Z/integratorMin"),
		get(n, "PIDs/Z/integratorMax"),
		"z")
	,m_pidYaw(
		get(n, "PIDs/Yaw/kp"),
		get(n, "PIDs/Yaw/kd"),
		get(n, "PIDs/Yaw/ki"),
		get(n, "PIDs/Yaw/kpp"),
		get(n, "PIDs/Yaw/minOutput"),
		get(n, "PIDs/Yaw/maxOutput"),
		get(n, "PIDs/Yaw/integratorMin"),
		get(n, "PIDs/Yaw/integratorMax"),
		"yaw")