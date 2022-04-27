#!/bin/bash
OUTFILE="import_list.inc"

echo $OUTFILE
echo '; Exported Shared Code from the Bootloader' >  $OUTFILE
echo '; ' >>$OUTFILE
echo '; See export_list for the list of addresses to export' >>$OUTFILE
echo '; Creates file import_list.inc which should be used by ' >>$OUTFILE
echo '; the application code.' >>$OUTFILE
echo '' >>$OUTFILE
echo 'ifdef USE_BOOTLOADER_CODE' >>$OUTFILE

while read srchstr; do 

   grep "$srchstr" *.lst | sed -e "s/:.*$//" -e"s/ 0000//" |awk '{if (NF==3) print $3 "\tequ\t0x" $1}'

done < export_list >> $OUTFILE
echo 'endif' >> $OUTFILE
echo '' >> $OUTFILE
grep "BOOTLOADER_SIZE" *.lst | awk '{if ((NF==5) && ($4=="equ")) print $3 "\tequ\t" $5}' >> $OUTFILE

grep "end_bank" *.lst | sed -e "s/:.*$//" -e"s/ 0000//" -e"s/end_/BL_/" | awk '{if (NF==3) print "#define " $3" 0x"$1}' >> $OUTFILE
