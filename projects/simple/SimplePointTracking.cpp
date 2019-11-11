#include "../../common_source/TriangulationPipeline.h"


/** @function main */
int main( int argc, char** argv )
{
    if( argc != 2 ){
        fprintf( stderr, "\n\nThe correct invocation of this program is \"SimplePointTracking configFile.json\"\n\n" );
        for( int i =0; i < argc; ++i){
            fprintf(stderr, "%s\n", argv[i]);
        }
        return 1;
    }
    InstallSignal(SIGSEGV);   // install our handler
    InstallSignal(SIGTERM);
    InstallSignal(SIGABRT);

    std::string config_file = argv[1];
    TriangulationPipeline* pipeline = TriangulationPipeline::Instance();
    return pipeline->Run( config_file );
}
