% Reference outputs of the ORIGINAL AirCrafter performance_modeller.m (unchanged) for the
% lap modeller code check (lap_mdo). Points span the inputs_file.m ranges. Writes tests/data/matlab_reference.csv.
% Run from a terminal:  matlab -batch "run('verification/matlab/run_reference.m')"
here = fileparts(mfilename('fullpath'));            % AirCrafter2/verification/matlab
repo = fileparts(fileparts(here));                  % AirCrafter2
addpath(fullfile(fileparts(repo), 'AirCrafter'));   % THESIS/AirCrafter (original MATLAB, unchanged)
outfile = fullfile(repo, 'tests', 'data', 'matlab_reference.csv');
assumptions_file;   % defines `assumptions`
inputs_file;        % defines `inputs` (takeoff distance, altitude, mission time, ranges)

pts = [ % m [kg], b [m], AR, V [m/s]
    5.0, 1.2, 6.0, 20.0;
    3.0, 0.9, 4.0, 15.0;
    7.0, 1.5, 9.0, 25.0;
    4.0, 1.5, 6.0, 18.0;
    6.0, 1.0, 5.0, 28.0;
    3.5, 1.3, 8.0, 12.0];
bfs = [1.0, 0.0];

fid = fopen(outfile, 'w');
fprintf(fid, 'm,b,AR,V,batteryFraction,NaNflag,Vstall,lapCount,Lap1time,otherLapTime,Tstatic,Preq,ThreeLapTime\n');
for k = 1:numel(bfs)
    for i = 1:size(pts, 1)
        in = inputs;
        in.m = pts(i,1); in.b = pts(i,2); in.AR = pts(i,3); in.V = pts(i,4);
        in.bannerL = 0; in.batteryFraction = bfs(k);
        out = performance_modeller(assumptions, in, []);
        if out.NaNflag
            fprintf(fid, '%.10g,%.10g,%.10g,%.10g,%g,1,NaN,NaN,NaN,NaN,NaN,NaN,NaN\n', pts(i,:), bfs(k));
        else
            fprintf(fid, '%.10g,%.10g,%.10g,%.10g,%g,0,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n', pts(i,:), bfs(k), ...
                out.Vstall, out.lapCount, out.Lap1time, out.otherLapTime, out.Tstatic, out.Preq, out.ThreeLapTime);
        end
    end
end
fclose(fid);
[~, ~, ~, rho] = atmosisa(inputs.altitude, "extended", "on");
fprintf('rho(%g m) = %.12g\n', inputs.altitude, rho);
type(outfile)
