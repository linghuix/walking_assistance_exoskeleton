function Z = HeightData(X ,Y)
	R = sqrt((X+5).^2 + (Y+5).^2);
	Z = sin(R)./R + 0.28;
end