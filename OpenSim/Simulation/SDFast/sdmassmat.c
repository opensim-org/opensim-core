#define rdNU 1
void rd_sdmassmat(double *aI)
{
	int i,j;
	double I[rdNU][rdNU];
	sdmassmat(I);

	for(i=0;i<rdNU;i++) {
		for(j=0;j<rdNU;j++,aI++) {
			*aI = I[i][j];
		}
	}
}
