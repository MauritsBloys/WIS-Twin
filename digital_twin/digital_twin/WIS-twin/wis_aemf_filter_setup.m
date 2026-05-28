%% wis_aemf_filter_setup.m
% Offline berekening van AEMF-filtermatrices voor WIS lekkagedetectie.
%
% Formulering:  H(q)*x + (L(q) + sum_j c_j * Lf_j) * z = 0
%
%   z = [u(3); y(3); y2(3); y3(3)]   12-dim bekend signaal
%     u    = regelaaringang
%     y    = waterstandmetingen [m]
%     y1_j = hoogteverschil kanaal j [cm]
%     y2_j = sqrt(y1_j)    [cm^0.5]   — alpha-basisfunctie
%     y3_j = y1_j^(3/2)   [cm^1.5]   — beta-basisfunctie
%
%   H(q) bevat alleen de nominale lineaire dynamica (geen lekkage).
%   Lf_j beschrijft hoe de j-de lekkageparameter z bemvloedt.
%   Hf_j = 0  (fout zit uitsluitend in L, niet in H).
%
% Uitvoer: data/wis_aemf_filter.mat  (laad eenmalig; digital_twin.m
%          detecteert dit bestand automatisch)
%
% Vereisten:
%   - distributed_workspace.mat  (WIS-sim/simulation/)
%   - MatrixPolynomial.m en barrify.m (aanwezig in WIS-twin/)

here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '../WIS-sim/simulation'));

%% ---- Systeemmatrices laden en discretiseren op 1 Hz ----
ws_file = fullfile(here, '../WIS-sim/simulation/distributed_workspace.mat');
assert(isfile(ws_file), ...
    'distributed_workspace.mat niet gevonden. Run generate_plant_workspace.m eerst.');
load(ws_file, 'comb_plant_cont');
pd = c2d(comb_plant_cont, 1, 'zoh');
A = pd.A;  B = pd.B;  C = pd.C;
nx = size(A,1);  nu = size(B,2);  ny = size(C,1);  % 12, 3, 3

wl_idx = arrayfun(@(i) find(abs(C(i,:)) > 0.5, 1), 1:3)';
area   = [0.1853, 0.1187, 0.2279];   % bassins 1-3 [m^2]

%% ---- Foutkanaalmatrices ----
% B_alpha(:,j) = effect van alpha_j * y2_j op toestanden
B_alpha = zeros(nx, 3);
B_alpha(wl_idx(1), 1) = +1/(1e6*area(1));
B_alpha(wl_idx(1), 2) = -1/(1e6*area(1));
B_alpha(wl_idx(2), 2) = +1/(1e6*area(2));
B_alpha(wl_idx(2), 3) = -1/(1e6*area(2));
B_alpha(wl_idx(3), 3) = +1/(1e6*area(3));
B_beta = B_alpha;

%% ---- MatrixPolynomials: nominale DAE ----
%
%   z = [u(3); y(3); y2(3); y3(3)]   nz = 12
%
%   Rijen 1-12 (toestandsvergelijking):
%       (q*I - A)*x  +  (-B)*u  =  0
%   Rijen 13-15 (uitvoervergelijking):
%       -C*x  +  y  =  0
%
nz = nu + 3*ny;   % 12

H = MatrixPolynomial( ...
    [-A; -C], ...
    [eye(nx); zeros(ny,nx)] );

%   L_0 = [-B, 0, 0, 0;    rijen 1-12
%           0,  I, 0, 0]    rijen 13-15
L0 = zeros(nx+ny, nz);
L0(1:nx,       1:nu)       = -B;
L0(nx+1:nx+ny, nu+1:nu+ny) =  eye(ny);
L = MatrixPolynomial(L0);

%% ---- Fout-L matrices (Hf_j = 0) ----
%   y2_j staat op kolom nu+ny+j   in z  (kolommen 7-9)
%   y3_j staat op kolom nu+2*ny+j in z  (kolommen 10-12)
Lf = cell(6, 1);
for j = 1:3
    Lf_alpha = zeros(nx+ny, nz);
    Lf_alpha(1:nx, nu+ny+j) = B_alpha(:, j);
    Lf{j} = MatrixPolynomial(Lf_alpha);

    Lf_beta = zeros(nx+ny, nz);
    Lf_beta(1:nx, nu+2*ny+j) = B_beta(:, j);
    Lf{j+3} = MatrixPolynomial(Lf_beta);
end

%% ---- Minimale filterorde ----
fprintf('Zoek minimale filterorde...\n');
order = 0;
while true
    Hb = H.barrify(order);
    IO = [eye(nx), zeros(nx, size(Hb,2) - nx)];
    Hd = lsqminnorm(Hb', IO')';
    if norm(Hd*Hb - IO, 'fro') < 1e-8 && ~isempty(null(Hb'))
        break
    end
    order = order + 1;
    assert(order <= 10, 'Observability criterion mislukt bij order %d.', order);
end
fprintf('Filterorde: %d\n', order);

%% ---- N uit nulruimte van H_bar ----
Hbar = H.barrify(order);
NR   = null(Hbar')';
assert(~isempty(NR), 'Nulruimte van Hbar'' is leeg.');
fprintf('Nulruimte dimensie: %d\n', size(NR, 1));
N_filt = MatrixPolynomial(NR, order);

%% ---- M_j = N * Lf_j  (G_j = Lf_j omdat Hf_j = 0) ----
M = cell(6, 1);
for j = 1:6
    M{j} = clean(N_filt * Lf{j});
end
fprintf('M-graad alpha ch1: %d,  beta ch1: %d\n', M{1}.Degree, M{4}.Degree);

%% ---- Compacte rijvorm opslaan ----
aemf.N_rows   = N_filt.rowForm;
aemf.N_degree = order;
aemf.m        = size(NR, 1);

aemf.Ma_rows  = cell(3,1);
aemf.Mb_rows  = cell(3,1);
aemf.Ma_deg   = zeros(3,1);
aemf.Mb_deg   = zeros(3,1);
for j = 1:3
    aemf.Ma_rows{j} = M{j}.rowForm;    aemf.Ma_deg(j) = M{j}.Degree;
    aemf.Mb_rows{j} = M{j+3}.rowForm;  aemf.Mb_deg(j) = M{j+3}.Degree;
end

aemf.nx = nx;  aemf.nu = nu;  aemf.ny = ny;  aemf.nz = nz;
aemf.A  = A;   aemf.B  = B;   aemf.C  = C;
aemf.B_alpha = B_alpha;  aemf.B_beta = B_beta;
aemf.wl_idx  = wl_idx;   aemf.area   = area;

out_dir = fullfile(here, 'data');
if ~isfolder(out_dir); mkdir(out_dir); end
out_file = fullfile(out_dir, 'wis_aemf_filter.mat');
save(out_file, 'aemf');
fprintf('Opgeslagen: %s\n', out_file);
