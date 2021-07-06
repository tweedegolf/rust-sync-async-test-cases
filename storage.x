
SECTIONS
{
    .foobar (NOLOAD) :
    {  
        . = ALIGN(4);
        . = . + 1024;
        . = ALIGN(4);
    } > PANDUMP
}
