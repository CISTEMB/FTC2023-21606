// Enums vs special meanings

// Special meanings
int colorNumber = 1; //(1 for blue, 2 for red, 3 for green)

// Use an enum
enum Color {
        BLUE,
		RED,        
		GREEN
    }
	
Color colorEnum = Color.RED;  // Color.RED for red, Color.BLUE for blue, Color.GREEN for green

// Example if statement
if (colorNumber == 1) {
	//  Do the blue thing
} else if (colorNumber == 2){
	//  Do the red thing
} else if (colorNumber == 3){
	// Do the green thing
}

// example enum thing
switch (colorEnum) {
	case BLUE:
		// Do blue thing
		break;
	case RED:
		// Do red thing
		break;
	case GREEN:
		// Do green thing
		break;
}


