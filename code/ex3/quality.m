%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Drum Masking Experiment (True Frequency Masking + PEAQ)
%
% - Uses STFT to analyze spectral peaks
% - Dynamically removes masked frequencies instead of predefined time windows
% - Runs PEAQ to compare perceived quality loss
% - Plots original vs. damaged waveform at the end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc; close all;

%% 0) ADD PEAQ PATHS (update paths as needed)
addpath('C:\Users\ROG\Desktop\ECE 241\project\PEAQ\PEAQ');
addpath('C:\Users\ROG\Desktop\ECE 241\project\PEAQ\PEAQ\PQevalAudio\Misc');
addpath('C:\Users\ROG\Desktop\ECE 241\project\PEAQ\PEAQ\PQevalAudio\CB');
addpath('C:\Users\ROG\Desktop\ECE 241\project\PEAQ\PEAQ\PQevalAudio\MOV');
addpath('C:\Users\ROG\Desktop\ECE 241\project\PEAQ\PEAQ\PQevalAudio\Patt');
addpath('C:\Users\ROG\Desktop\ECE 241\project\PEAQ\PEAQ\PQevalAudio');

%% USER PARAMETERS
audioFile = '779199__crashbulb__glitch-hop-spacey-loop-slide-guitar-bass-drums-96-bpm-trip-hop-rock.wav';

targetFs   = 48000;   % Standardize to 48 kHz
bitDepth   = 16;

% Snippet range: 0.0 - 5.0 s
snippetStartSec = 0.0;
snippetDurSec   = 5.0;

% Frequency ranges for bandstop filter (for kick drum masking)
fLow  = [80, 160];    % Adjusted LF range

%% 1) LOAD & (OPTIONALLY) RESAMPLE TO 48kHz, 16-BIT
[raw, fsIn] = audioread(audioFile);
rawMono = mean(raw, 2);  % Convert stereo to mono

fprintf('Loaded: %s\n', audioFile);
fprintf('   Original sample rate: %d Hz\n', fsIn);
fprintf('   Duration: %.2f seconds\n', length(rawMono)/fsIn);

if fsIn ~= targetFs
    rawMono = resample(rawMono, targetFs, fsIn);
    fsIn = targetFs;
    fprintf('Resampled to %d Hz.\n', fsIn);
end

% Convert to 16-bit integer range
rawInt16 = int16(rawMono * 32767);
orig48File = 'DrumMask_Original_48kHz16bit.wav';
audiowrite(orig48File, rawInt16, fsIn, 'BitsPerSample', bitDepth);

% Reload as float [-1..1]
[origAudio, fs] = audioread(orig48File);
nSamples = length(origAudio);

%% 2) EXTRACT SNIPPET (0.0-5.0 s)
snippetStart = round(snippetStartSec*fs) + 1;
snippetEnd   = snippetStart + round(snippetDurSec*fs) - 1;
snippetEnd   = min(snippetEnd, nSamples);

snippetAudio = origAudio(snippetStart:snippetEnd);
tSnippet = (0:length(snippetAudio)-1)' / fs;
snippetLen = length(snippetAudio);

% Save original snippet
snippetCleanFile = 'DrumMask_OriginalSnippet.wav';
audiowrite(snippetCleanFile, snippetAudio, fs, 'BitsPerSample', bitDepth);

%% 3) AUTOMATIC TEMPORAL MASKING (LOW FREQ) BASED ON ENERGY
% Design bandstop filter for low-frequency masking
dLow = designfilt('bandstopiir', 'FilterOrder', 6, ...
    'HalfPowerFrequency1', fLow(1), 'HalfPowerFrequency2', fLow(2), ...
    'SampleRate', fs);

% Compute short-time RMS energy envelope using a 20-ms window
winSize = round(0.02 * fs);  % 20 ms window length
energyEnvelope = sqrt(movmean(snippetAudio.^2, winSize));

% Set energy threshold (adjust this fraction as needed)
energyThreshold = 0.5 * max(energyEnvelope);

% Identify low-energy segments where the energy is below the threshold
lowEnergyIdx = energyEnvelope < energyThreshold;

% Group contiguous low-energy indices into segments
d = diff([0; lowEnergyIdx; 0]);
autoStartIdx = find(d==1);
autoEndIdx = find(d==-1)-1;

% Filter out segments that are too short (e.g., less than 50 ms)
minDuration = 0.05; % seconds
minSamples = round(minDuration * fs);
validSegments = (autoEndIdx - autoStartIdx + 1) >= minSamples;
autoStartIdx = autoStartIdx(validSegments);
autoEndIdx = autoEndIdx(validSegments);

% (Optional) Display detected temporal masking windows in seconds:
autoWindows = [ ((autoStartIdx-1)/fs)+0.02, ((autoEndIdx-1)/fs)+0.02 ];
disp('Automatically detected temporal masking windows (sec):');
disp(autoWindows);

% Apply bandstop filtering to each detected low-energy segment
damagedSnippet = snippetAudio; % initialize with original snippet
for w = 1:length(autoStartIdx)
    damagedSnippet(autoStartIdx(w):autoEndIdx(w)) = filtfilt(dLow, damagedSnippet(autoStartIdx(w):autoEndIdx(w)));
end

%% 4) APPLY TRUE FREQUENCY MASKING USING A PSYCHOACOUSTIC MODEL
nFFT = 1024;  
overlap = 512;
window = hamming(nFFT);

% Compute spectrogram of the damaged snippet (after temporal masking)
[S, F, T] = spectrogram(damagedSnippet, window, overlap, nFFT, fs);

% Masking Threshold = 20 dB lower than peak energy in each time frame
maskThreshold = max(abs(S), [], 1) - 20;  

% Attenuate frequency bins below the threshold in each time frame by 50%
for frame = 1:size(S,2)
    maskFreqs = abs(S(:,frame)) < db2mag(maskThreshold(frame));
    S(maskFreqs, frame) = S(maskFreqs, frame) * 0.5;
end

% Inverse STFT to reconstruct the damaged snippet
damagedSnippet = istft(S, fs, 'Window', hamming(nFFT), 'OverlapLength', overlap, ...
    'FFTLength', nFFT, 'FrequencyRange', 'onesided');

%% 5) SAVE DAMAGED SNIPPET
damagedSnippetInt16 = int16(damagedSnippet * 32767);
snippetDamagedFile  = 'DrumMask_DamagedSnippet_FreqMask.wav';
audiowrite(snippetDamagedFile, damagedSnippetInt16, fs, 'BitsPerSample', bitDepth);

%% 6) RUN PEAQ TO COMPARE QUALITY LOSS
fprintf('\n-- Running PEAQ to Compare Perceived Quality Loss --\n');
try
    [odg, movb] = PQevalAudio_fn(snippetCleanFile, snippetDamagedFile);
    fprintf('PEAQ Objective Difference Grade (ODG): %.2f\n', odg);
catch ME
    warning('PEAQ evaluation failed: %s', E.message);
end

%% 7) PLOT ORIGINAL vs. DAMAGED SNIPPET
tDamaged = (0:length(damagedSnippet)-1)' / fs;
figure('Position', [100, 100, 1000, 600]); 
plot(tSnippet, snippetAudio, 'b', 'LineWidth', 1.2); hold on;
plot(tDamaged, damagedSnippet, 'r', 'LineWidth', 1.2);
xlabel('Time (s)', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Amplitude', 'FontSize', 14, 'FontWeight', 'bold');
title({'Original vs. Combined Masked Snippet'}, 'FontSize', 16, 'FontWeight', 'bold');
legend('Original', 'Combined', 'Location', 'Best');
grid on;
xlim([0 max(tSnippet)]);

fprintf('\nPlot generated: Compare the original vs. combined (temp+freq) masked snippet visually.\n');
