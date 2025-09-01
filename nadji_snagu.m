function [trazena_snaga, frekvencija_talasa] = nadji_snagu(f,P, donja_granica, gornja_granica)
        opseg = (f>=donja_granica) .* (f<=gornja_granica);
        indeksi = opseg.*(1:length(P));
        indeksi(indeksi==0)=[];
        trazena_snaga = P(indeksi);
        frekvencija_talasa = f(indeksi);
end

