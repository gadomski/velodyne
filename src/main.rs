extern crate docopt;
extern crate rustc_serialize;
extern crate velodyne;

use docopt::Docopt;
use velodyne::io::{Read, Pcap};

const USAGE: &'static str = "
Usage: velodyne info <infile>
";

#[derive(Debug, RustcDecodable)]
struct Args {
    cmd_info: bool,
    arg_infile: String,
}

fn main() {
    let args: Args = Docopt::new(USAGE).and_then(|d| d.decode()).unwrap_or_else(|e| e.exit());
    if args.cmd_info {
        let pcap = Pcap::open(args.arg_infile).unwrap();
        let mut npoints = 0;
        for packet in pcap.vlp_16_packets().map(|result| result.unwrap()) {
            npoints += packet.points().unwrap().len();
        }
        println!("Points: {}", npoints);
    }
}
