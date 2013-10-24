 dirspec='/home/ssafarik/FlylabData/2013_10_10*'; 
 filespec='zapresponse_TrpA1_paralysis_30s_03s_117s_260mW_*.csv'; 
 interactions=FlylabGetInteractionsFiles(dirspec,filespec, 0, 2, 10, 'all'); 
 FlylabPlotAllInteractions(interactions, 0, 0, 6);
 