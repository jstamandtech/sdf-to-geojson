//#![feature( array_chunks)]

#[macro_use]
extern crate nom;

use glob::glob;

use std::fs::File;
use std::io::Read;
use std::slice;
use byteorder::{LittleEndian, ReadBytesExt};
use nom::{IResult, bytes::complete::{tag, take_while_m_n}, combinator::map_res, sequence::tuple};

use nom::bytes::complete::take_until;
use nom::combinator::map_opt;
use nom::number::complete::le_u32;
use std::mem::take;
use std::f64::consts::PI;
use nom_derive::{Nom, Parse};
use nom::Err;

use float_cmp::{ApproxEq, F64Margin};

use geojson::{FeatureCollection, GeoJson, Feature, Value, Position, LineStringType};
use geo_types::{GeometryCollection, LineString, Polygon, Geometry};


//used to mark the beginning of a ping in the data stream
const PING_MARKER: &[u8] = &[0xFF, 0xFF, 0xFF, 0xFF];

use byte_slice_cast::*;
//use std::slice::range;
use std::string::ParseError;
use std::alloc::System;
use std::iter::FromIterator;

//not currently here for now but may be encountered somewhere
enum KleinErrors {
    InvalidSpeed,
    GpsDataError,
    TelemetryError,
    SensorChecksumError,
    TowfishLeak,
    BadData,
    Watchdog,
    CompassError,
    NoGpsLatlonSentInput,
    NoGpsSpeedSentInput,
    NoGpsZdaSentInput,
    MruError,
    No1PpsInput,
}


#[derive(Debug, Nom)]
#[nom(LittleEndian)]
pub struct DataPageHeader {
    numberBytes: u32,
    pageVersion: u32,
    configuration: u32,
    pingNumber: u32,
    numSamples: u32,
    beamsToDisplay: u32,
    errorFlags: u32,
    range: u32,
    speedFish: u32,
    speedSound: u32,
    resMode: u32,
    txWaveform: u32,
    respDiv: u32,
    respFreq: u32,
    manualSpeedSwitch: u32,
    despeckleSwitch: u32,
    speedFilterSwitch: u32,
    year: u32,
    month: u32,
    day: u32,
    hour: u32,
    minute: u32,
    second: u32,
    hsecond: u32,
    fixTimeHour: u32,
    fixTimeMinute: u32,
    fixTimeSecond: f32,
    heading: f32,
    pitch: f32,
    roll: f32,
    depth: f32,
    altitude: f32,
    temperature: f32,
    speed: f32,
    shipHeading: f32,
    magneticVariation: f32,
    shipLat: f64,
    shipLon: f64,
    fishLat: f64,
    fishLon: f64,
    tvgPage: u32,
    headerSize: u32,
    fixTimeYear: u32,
    fixTimeMonth: u32,
    fixTimeDay: u32,
    auxPitch: f32,
    auxRoll: f32,
    auxDepth: f32,
    auxAlt: f32,
    cableOut: f32,
    fseconds: f32,
    altimeter: u32,
    sampleFreq: u32,
    depressorType: u32,
    cableType: u32,
    shieveXoff: f32,
    shieveYoff: f32,
    ShieveZoff: f32,
    GPSheight: f32,
    rawDataConfig: u32,
    header3ExtensionSize: u32,
    sbpTxWaveform: u32,
    sbpPreAmpGain: u32,
    sbpDataRaw: u32,
    sbpNumSamples: u32,
    sbpSampleFreq: u32,
    sbpTxWaveformVersion: u32,
    wingAngle: u32,
    emergencySwitchState: u32,
    laybackMethod: u32,
    laybackFishlat: f64,
    laybackFishLon: f64,
    fishHeadingOffset: f32,
    pressureSensorOffset: f32,

    //vxWorks V6.13
    tpuSwVersion: u32,

    //vxWorks V6.17
    capabilityMask: u32,

    //vxWorks V6.19
    txVersion: u32,

    //vxWorks V6.22
    numSamplesExtra: u32,

    //vxWorks V7.00
    postProcessVersion: u32,
    motionSensorType: u16,
    pingTimeRefCount: u16,
    extTrigTimeRefCount: u16,
    onePpsTimeRefCount: u16,
    timeRefCountWeight: u32,
    altitudeBathy: f32,
    pingInterval: f32,

    //vxWorks V8.00
    sdfExtensionSize: u32,
    secondsOfWeek: f64,
    speedSoundSource: u32,
    pressureSensorMax: f32,
    pressureSensorVoltageMin: f32,
    pressureSensorVoltageMax: f32,
    processedPingNumber: u32,
    processedPingSpacing: f32,
    temperateAmbient: f32,
    saturationDetectThreshold: i32,
    sonarFreq: u32,
    reserved: [u32; 26],
}


pub struct System3000DataChannels {
    portlf: Option<Vec<u16>>,
    stbdlf: Option<Vec<u16>>,
    porthf: Option<Vec<u16>>,
    stbdhf: Option<Vec<u16>>,
    sbp: Option<Vec<u32>>
}

pub struct System3000Sample{
    header: DataPageHeader,
    data: System3000DataChannels
}


fn take_sys3000_sample(b: &[u8]) -> IResult<&[u8], System3000Sample> {
    let mut bytes: &[u8] = b;
    bytes = take_until_ping_marker(bytes).unwrap().0;

    // take the data page header
    let mut header: Option<DataPageHeader> = None;
    let mut result = DataPageHeader::parse(bytes).expect("Error parsing data page header");
    bytes = match result {
        (b, header_data) => {
            header = Some(header_data);
            b
        },
        _ => {bytes}
    };

    //take the data channels
    let mut data_channels: Option<System3000DataChannels> = None;
    let mut data_result = take_sys3000_data_channels(bytes);
    bytes = match data_result {
        Ok((b, data)) => {
            data_channels = Some(data);
            b
        },
        _ => {bytes}
    };

    Ok((bytes,
    System3000Sample{
        header: header.take().unwrap(),
        data: data_channels.take().unwrap(),
    }
    ))

}

fn take_sys3000_data_channels(b: &[u8]) -> IResult<&[u8], System3000DataChannels> {

    let mut bytes: &[u8] = b;
    let mut result: Option<IResult<&[u8], Vec<u16>>> = None;
    let mut sbp_result: Option<IResult<&[u8], Vec<u32>>> = None;

    let mut portlf_samples: Option<Vec<u16>> = None;
    let mut stbdlf_samples: Option<Vec<u16>> = None;
    let mut porthf_samples: Option<Vec<u16>> = None;
    let mut stbdhf_samples: Option<Vec<u16>> = None;
    let mut sbp_samples: Option<Vec<u32>> = None;

    //port lf samples
    result = Some(nom::multi::length_count(nom::number::complete::le_u16, nom::number::complete::le_u16)(bytes));
    bytes = match result.unwrap() {
        Ok((b, v)) => {
            portlf_samples = Some(v);
            b
        }
        _ => {b}
    };

    //stbd lf samples
    result = Some(nom::multi::length_count(nom::number::complete::le_u16, nom::number::complete::le_u16)(bytes));
    bytes = match result.unwrap() {
        Ok((b, v)) => {
            stbdlf_samples = Some(v);
            b
        }
        _ => {b}
    };

    //porthf samples
    result = Some(nom::multi::length_count(nom::number::complete::le_u16, nom::number::complete::le_u16)(bytes));
    bytes = match result.unwrap() {
        Ok((b, v)) => {
            porthf_samples = Some(v);
            b
        }
        _ => {b}
    };

    //stbdhf samples
    result = Some(nom::multi::length_count(nom::number::complete::le_u16, nom::number::complete::le_u16)(bytes));
    bytes = match result.unwrap() {
        Ok((b, v)) => {
            stbdhf_samples = Some(v);
            b
        }
        _ => {b}
    };

    //subbottom profiler samples
    sbp_result = Some(nom::multi::length_count(nom::number::complete::le_u32, nom::number::complete::le_u32)(bytes));
    bytes = match sbp_result.unwrap() {
        Ok((b, v)) => {
            sbp_samples = Some(v);
            b
        }
        _ => {b}
    };

    Ok((bytes,
        System3000DataChannels{
        portlf: portlf_samples.take(),
        stbdlf: stbdlf_samples.take(),
        porthf: porthf_samples.take(),
        stbdhf: stbdhf_samples.take(),
        sbp: sbp_samples.take(),
    }))
}

fn take_until_ping_marker(b: &[u8]) -> IResult<&[u8], u32> {
    let result: IResult<&[u8], &[u8]> = take_until(PING_MARKER)(b);
    nom::number::complete::le_u32(result.unwrap().0)
}


fn main() {

    let mut files = Vec::new();
    for entry in glob("/home/jstamand/Documents/sonar_samples/*.sdf").expect("Failed to read glob"){
        match entry {
            Ok(path) => files.push(path),
            Err(e) => println!("{:?}", e),
        }
    }
    files.sort();

    let mut positions: LineStringType = Vec::new();
    for file in files {
        println!("Opening: {:?}", file.display());
        let mut f = File::open(&file).expect("File Error!");
        let metadata = File::metadata(&f).expect("no file found");
        let mut buffer: Vec<u8> = vec![0; metadata.len() as usize];
        f.read(&mut buffer).expect("buffer overflow");

        let mut slice: &[u8] = buffer.as_slice().as_byte_slice();

        let result_vec = nom::multi::many_till(take_sys3000_sample, nom::combinator::eof)(slice);

        let coord_margin: F64Margin = F64Margin{epsilon: 1e-3, ulps: 2};

        for sample in result_vec.unwrap().1.0 {
            let lat: f64 = sample.header.shipLat*180.0/PI;
            let lon: f64 = sample.header.shipLon*180.0/PI;

            match positions.last() {
                Some(position) => {
                    if !lon.approx_eq(position[0], coord_margin) || !lat.approx_eq(position[1], coord_margin){
                        positions.push(vec![lon, lat]);
                    }
                }
                _ => {
                    positions.push(vec![lon, lat]);
                }
            }
        }

    }

    let geometry: geojson::Geometry = geojson::Geometry::new(geojson::Value::LineString(positions));

    let feature = geojson::Feature{
        bbox: None,
        geometry: Some(geometry),
        id: None,
        properties: None,
        foreign_members: None
    };

    let fc = geojson::FeatureCollection{
        bbox: None,
        features: vec![feature],
        foreign_members: None
    };

    let geojson_string = fc.to_string();

    //TODO: clean this up
    let write_result = std::fs::write("/home/jstamand/Documents/temp/sonar_track.geojson", geojson_string);


}
