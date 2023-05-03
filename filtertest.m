

data = csvread('FT_data_horisontal_with_external_force.csv');


% All frequency values are in Hz.
Fs = 500;  % Sampling Frequency

Fstop = 5;           % Stopband Frequency
Fpass = 10;          % Passband Frequency
Astop = 80;          % Stopband Attenuation (dB)
Apass = 1;           % Passband Ripple (dB)
match = 'passband';  % Band to match exactly

% Construct an FDESIGN object and call its BUTTER method.
h  = fdesign.highpass(Fstop, Fpass, Astop, Apass, Fs);
Hd = design(h, 'butter', 'MatchExactly', match);
x = data(:,3);
t = (0:length(x)-1)/Fs;
y = filter(Hd,x);

plot(t,y)


%subplot(1,2,2)
%plot(data(:,1))

[1, -5.75724419, 13.81551081, -17.68737618, 12.74161733,  -4.89692489,   0.78441718]