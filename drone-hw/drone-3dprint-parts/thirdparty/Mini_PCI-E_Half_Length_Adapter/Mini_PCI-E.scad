difference() {

	union(){
		cube(size = [29,33,1]);

		translate([2.5,30,0.5])
		{ cylinder(h=3, r=1.5, $fn=20); }

		translate([26.5,30,0.5])
		{ cylinder(h=3, r=1.5, $fn=20); }
	}

	translate([4.5,26,-0.5])
	{ cube(size = [20,7.5,2]); }

	translate([3,4.5,-0.5])
	{ cylinder(h=2, r=1.25, $fn=20); }

	translate([26.5,4.5,-0.5])
	{ cylinder(h=2, r=1.25, $fn=20); }
}
